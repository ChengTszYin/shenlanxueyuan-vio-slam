#include <pangolin/pangolin.h>
#include <vector>
#include <fstream>
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "sophus/se3.h"
#include <unistd.h>
#include <cmath>
using namespace std;

string compare_file = "./compare.txt";

typedef vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> TrajectoryType;
TrajectoryType traj_g, traj_e;

double time_e = 0.0, tx_e = 0.0, ty_e = 0.0, tz_e = 0.0, q1_e = 0.0, q2_e = 0.0, q3_e = 0.0, q4_e = 0.0,
        time_q = 0.0, tx_g = 0.0, ty_g = 0.0, tz_g = 0.0, q1_g = 0.0, q2_g = 0.0, q3_g = 0.0, q4_g = 0.0;

void DrawTrajectory(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3> > ground_truth_poses, vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3> > estimated_poses)
{
    if (ground_truth_poses.empty() || estimated_poses.empty()) {
        cerr << "Trajectory is empty!" << endl;
        return;
    }

    // create pangolin window and plot the trajectory
    pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));


    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        glLineWidth(2);
        for (size_t i = 0; i < ground_truth_poses.size() - 1; i++) {
            glColor3f(0.0f, 0.0f, 1.0f);
            glBegin(GL_LINES);
            auto p1 = ground_truth_poses[i], p2 = ground_truth_poses[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();

            glColor3f(1.0f, 0.0f, 0.0f);
            glBegin(GL_LINES);
            p1 = estimated_poses[i], p2 = estimated_poses[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }
        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }
}

int main(int argc, char **argv) {

    // read compare.txt
    ifstream com_f(compare_file);
    string line;
    while(getline(com_f, line))
    {
        if(line.empty()) continue;
        stringstream ss(line);
        if(ss >> time_e >> tx_e >> ty_e >> tz_e >> q1_e >> q2_e >> q3_e >> q4_e >> 
                time_q >> tx_g >> ty_g >> tz_g >> q1_g >> q2_g >> q3_g >> q4_g)
        {
            Sophus::SE3 p1(Eigen::Quaterniond(q1_e, q2_e, q3_e, q4_e), Eigen::Vector3d(tx_e, ty_e, tz_e));
            Sophus::SE3 p2(Eigen::Quaterniond(q1_g, q2_g, q3_g, q4_g), Eigen::Vector3d(tx_g, ty_g, tz_g));
            traj_g.push_back(p1);
            traj_e.push_back(p2);
        }
    }
    // calculate the rmse
    double rmse = 0;
    for(size_t i = 0; i < traj_g.size(); ++i)
    {
        Sophus::SE3 actu = traj_g[i], pred = traj_e[i];
        double error = (actu.inverse() * pred).log().norm();
        rmse += error * error;
    }

    rmse = sqrt(rmse / traj_g.size());
    cout << "rmse: " << rmse << endl;
    // find the centroid of pcl
    Eigen::Vector3d centroid_g = Eigen::Vector3d::Zero();
    for(auto& pose: traj_g)
    {
        centroid_g += pose.translation();
    }
    centroid_g = centroid_g / traj_g.size();

    Eigen::Vector3d centroid_e = Eigen::Vector3d::Zero();
    for(auto& pose: traj_e)
    {
        centroid_e += pose.translation();
    }
    centroid_e = centroid_e / traj_e.size();
    
    //remove center
    vector<Eigen::Vector3d> q_g(traj_g.size()), q_e(traj_e.size());
    for(size_t i = 0; i < q_g.size(); ++i)
    {
        q_g[i] = traj_g[i].translation() - centroid_g;
    }
    for(size_t j = 0; j < q_e.size(); ++j)
    {
        q_e[j] = traj_e[j].translation() - centroid_e;
    }
    // compute q1*q2^T
    Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
    for(size_t k = 0; k < traj_g.size(); ++k)
    {
        W += Eigen::Vector3d(q_g[k].x(), q_g[k].y(), q_g[k].z()) * Eigen::Vector3d(q_e[k].x(), q_e[k].y(), q_e[k].z()).transpose();
    }
    cout << "W:" << endl << W << endl;
    // SVD on W
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();

    cout << "U=" << U << endl;
    cout << "V=" << V << endl;

    Eigen::Matrix3d R_ = U * (V.transpose());
    if(R_.determinant() < 0){
        R_ = -R_;
    }
    Eigen::Vector3d t_ = Eigen::Vector3d(centroid_g.x(), centroid_g.y(), centroid_g.z()) - R_ * Eigen::Vector3d(centroid_e.x(), centroid_e.y(), centroid_e.z());

    Eigen::Matrix3d R;
    R << R_(0, 0), R_(0, 1), R_(0, 2),
         R_(1, 0), R_(1, 1), R_(1, 2),
         R_(2, 0), R_(2, 1), R_(2, 2);
    Eigen::Vector3d t = t_;
    cout << "R:" << endl << R << endl;
    cout << "t:" << endl << t << endl;
    Sophus::SE3 align(R, t);
    TrajectoryType update_traj_e = traj_e;
    for(size_t l = 0; l < update_traj_e.size(); ++ l)
    {
        update_traj_e[l] = align * traj_e[l]; 
    }
    //draw the trajectory before icp
    DrawTrajectory(traj_g, traj_e);
    // DrawTrajectory(traj_g, update_traj_e);
    return 0;
}
