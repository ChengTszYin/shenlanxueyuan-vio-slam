#include <iostream>
#include "common.h"
#include <Eigen/Core>
#include <sophus/se3.h>
#include <sophus/so3.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <memory>
using namespace std;

string bal_file = "/home/dllm/Documents/deepblue/code_prac/p8/data.txt";

struct CameraPara
{
    CameraPara()
    {
        rotation = Sophus::SO3();
        translation = Eigen::Vector3d::Zero();
        focal = 0;
        k1 = 0;
        k2 = 0;
    };

    explicit CameraPara(double *data_addr)
    {
        rotation = Sophus::SO3::exp(Eigen::Vector3d(data_addr[0], data_addr[1], data_addr[2]));
        translation = Eigen::Vector3d(data_addr[3], data_addr[4], data_addr[5]);
        focal = data_addr[6];
        k1 = data_addr[7];
        k2 = data_addr[8];
    }

    void set_to(double *data_addr)
    {
        auto r = rotation.log();
        for(int i = 0; i < 3; ++i) data_addr[i] = r[i];
        for(int i = 0; i < 3; ++i) data_addr[i + 3] = translation[i];
        data_addr[6] = focal;
        data_addr[7] = k1;
        data_addr[8] = k2;
    }

    Sophus::SO3  rotation;
    Eigen::Vector3d translation = Eigen::Vector3d::Zero();
    double focal = 0;
    double k1 = 0, k2 = 0;
};

class CameraVertex : public g2o::BaseVertex<9, CameraPara>
{
    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    CameraVertex() {};

    virtual void setToOriginImpl () override {      // reset the value of parameters
        _estimate = CameraPara();
    };

    virtual void oplusImpl (const double *update) override {    // update the value of parameter during optimization
        _estimate.rotation = Sophus::SO3::exp(Eigen::Vector3d(update[0], update[1], update[2])) * _estimate.rotation;
        _estimate.translation += Eigen::Vector3d(update[3], update[4], update[5]);
        _estimate.focal += update[6];
        _estimate.k1 += update[7];
        _estimate.k2 += update[8];
    };

    Eigen::Vector2d project(const Eigen::Vector3d &point)
    {
        Eigen::Vector3d pc = _estimate.rotation * point + _estimate.translation;
        Eigen::Vector3d p_dot = -1 * pc / pc[2];
        double r2 = p_dot.squaredNorm();
        double distortion = 1.0 + r2 * (_estimate.k1 + _estimate.k2 * r2);
        return Eigen::Vector2d(_estimate.focal * distortion * p_dot[0], _estimate.focal * distortion * p_dot[1]);
    }  

    virtual bool read(std::istream &in) {};
    virtual bool write(std::ostream &out) const {};
};

class BVertex : public g2o::BaseVertex<3, Eigen::Vector3d>   // wrap 3d point X Y Z into vertex
{
    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    BVertex(){}; //create an empty point vertex

    virtual void setToOriginImpl () override{   //set that point to the origin
        _estimate = Eigen::Vector3d(0, 0, 0);
    };

    virtual void oplusImpl (const double *update){  //LM gives 3 numbers; add them to $X,Y,Z$
        _estimate += Eigen::Vector3d(update[0], update[1], update[2]);
    }

    virtual bool read(std::istream &in) {};
    virtual bool write(std::ostream &out) const {};
};

class BBinaryEdge: public g2o::BaseBinaryEdge<2, Eigen::Vector2d, CameraVertex, BVertex>
{
    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    virtual void computeError() override 
    {
        auto v0 = (CameraVertex *) _vertices[0];
        auto v1 = (BVertex *) _vertices[1];
        auto proj = v0 -> project(v1->estimate());
        _error = proj - _measurement;
    }

    virtual bool read(istream &in){}

    virtual bool write(ostream &out) const {}
};

void solveBA(BALProblem &bal)
{
    const int point_block_size = bal.point_block_size();
    const int camera_block_size = bal.camera_block_size();
    double *points = bal.mutable_points();
    double *cameras = bal.mutable_cameras();

    typedef g2o::BlockSolver<g2o::BlockSolverTraits<9,3>> BlockSolverType;
    typedef g2o::LinearSolverCSparse<BlockSolverType::PoseMatrixType> LinearSolverType;

    auto solver = new g2o::OptimizationAlgorithmLevenberg(
        std::make_unique<BlockSolverType>(std::make_unique<LinearSolverType>())
    );
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(true);

    const double *observations = bal.observations();

    vector<CameraVertex *> camera_vetex;
    vector<BVertex *> base_vertex;
    for(int i = 0; i < bal.num_cameras(); ++i)
    {
        CameraVertex *v = new CameraVertex();
        double *camera = cameras + camera_block_size * i;
        v->setId(i);
        v->setEstimate(CameraPara(camera));
        optimizer.addVertex(v);
        camera_vetex.push_back(v);
    } 

    for(int i = 0; i < bal.num_points(); ++i)
    {
        BVertex *v = new BVertex();
        double *point = points + point_block_size * i;
        v->setId(i + bal.num_cameras());
        v->setEstimate(Eigen::Vector3d(point[0], point[1], point[2]));
        v->setMarginalized(true);
        optimizer.addVertex(v);
        base_vertex.push_back(v);
    }

    for(int i = 0; i < bal.num_observations(); ++i)
    {
        BBinaryEdge *edge = new BBinaryEdge();
        edge->setVertex(0, camera_vetex[bal.camera_index()[i]]);
        edge->setVertex(1, base_vertex[bal.point_index()[i]]);
        edge->setMeasurement(Eigen::Vector2d(observations[2 * i + 0], observations[2 * i + 1]));
        edge->setInformation(Eigen::Matrix2d::Identity());
        edge->setRobustKernel(new g2o::RobustKernelHuber());
        optimizer.addEdge(edge);
    }
    
    optimizer.initializeOptimization();
    optimizer.optimize(15);

    for(int i = 0; i < bal.num_cameras(); ++i)
    {
        double *camera = cameras + camera_block_size * i;
        auto vertex = camera_vetex[i];
        auto estimate = vertex->estimate();
        estimate.set_to(camera);
    }

    for(int i = 0; i < bal.num_points(); ++i)
    {
        double *point = points + point_block_size * i;
        auto vertex = base_vertex[i];
        for(int k = 0; k < 3; ++k) point[k] = vertex ->estimate()[k];
    }
}

int main(int argc, char **argv)
{
    BALProblem bal(bal_file, false);
    int num_cameras = bal.num_cameras(), num_points = bal.num_points(), num_observations = bal.num_observations();
    cout << "num_cameras: " << num_cameras << " num_points: " << num_points << " num_observations: " << num_observations << endl;
    bal.Normalize();
    bal.WriteToPLYFile("initial.ply");
    solveBA(bal);
    bal.WriteToPLYFile("final.ply");
    return 0;
}