//
// Created by xiang on 12/21/17.
//

#include <Eigen/Core>
#include <Eigen/Dense>

using namespace Eigen;

#include <vector>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include "sophus/se3.h"
using namespace std;

typedef vector<Vector3d, Eigen::aligned_allocator<Vector3d>> VecVector3d;
typedef vector<Vector2d, Eigen::aligned_allocator<Vector3d>> VecVector2d;
typedef Matrix<double, 6, 1> Vector6d;

string p3d_file = "./p3d.txt";
string p2d_file = "./p2d.txt";

void bundleAdjustment()
{

}

int main(int argc, char **argv) {

    VecVector2d p2d;
    VecVector3d p3d;
    Matrix3d K;
    double fx = 520.9, fy = 521.0, cx = 325.1, cy = 249.7;
    K << fx, 0, cx, 0, fy, cy, 0, 0, 1;

    // load points in to p3d and p2d 
    // START YOUR CODE HERE
    ifstream p3d_f(p3d_file);
    ifstream p2d_f(p2d_file);
    if (!p3d_f.is_open() || !p2d_f.is_open()) {
        cerr << "Error: Could not open the file!" << std::endl;
        return 1;
    }

    string line;
    while (getline(p3d_f, line)) {
        if(line.empty()) continue;
        stringstream ss(line);
        Eigen::Vector3d pts;
        if(ss >> pts[0] >> pts[1] >> pts[2])
        {
            p3d.push_back(pts);
        }
    }

    while (getline(p2d_f, line)) {
        if(line.empty()) continue;
        stringstream ss(line);
        Eigen::Vector2d pts;
        if(ss >> pts[0] >> pts[1])
        {
            p2d.push_back(pts);
        }
    }

    // END YOUR CODE HERE
    assert(p3d.size() == p2d.size());

    vector<cv::Point3d> p3d_;
    vector<cv::Point2d> p2d_;
    for(size_t i=0; i < p3d.size(); ++i){
        p3d_.emplace_back(p3d[i][0], p3d[i][1], p3d[i][2]);
    }
    for(size_t i=0; i < p2d.size(); ++i){
        p2d_.emplace_back(p2d[i][0], p2d[i][1]);
    }
    cv::Mat _K;
    cv::eigen2cv(K, _K);
    int iterations = 100;
    double cost = 0;
    double lastCost = 0;
    int nPoints = p3d.size();
    cout << "points: " << nPoints << endl;
    cv::Mat r, t;
    cv::solvePnP(p3d_, p2d_, _K, cv::Mat(), r, t, false);
    cv::Mat R;
    cv::Rodrigues(r, R);
    cout << "R=" << endl << R << endl;
    cout << "t=" << endl << t << endl;

    Eigen::Matrix3d eigen_r;
    Eigen::Vector3d eigen_t;
    
    eigen_r << R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2),
                R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2),
                R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2,2);

    eigen_t << t.at<double>(0), t.at<double>(1), t.at<double>(2);
    Sophus::SE3 T_esti(eigen_r, eigen_t); // estimated pose

    for (int iter = 0; iter < iterations; iter++) {

        Matrix<double, 6, 6> H = Matrix<double, 6, 6>::Zero();
        Vector6d b = Vector6d::Zero();

        cost = 0;
        // compute cost
        for (int i = 0; i < nPoints; i++) {
        // compute cost for p3d[I] and p2d[I]
        // START YOUR CODE HERE 
            Eigen::Vector4d P, P_prim;
            P << p3d[i][0], p3d[i][1], p3d[i][2], 1.0;
            P_prim = T_esti.matrix() * P;       // 4 x 1 matrix [R | t] * P
            double inv_z = 1.0 / P_prim[2];
            double inv_z2 = inv_z * inv_z;
            double u, v;
            u = fx * P_prim[0] * inv_z + cx;
            v = fy * P_prim[1] * inv_z + cy;
            Eigen::Vector2d e(p2d[i][0] - u, p2d[i][1] - v);
            cost += e.squaredNorm();

	    // END YOUR CODE HERE

	    // compute jacobian
            Matrix<double, 2, 6> J;
            // START YOUR CODE HERE 
            J << fx * inv_z,
                0, 
                -fx * P_prim[0] * inv_z2,
                -fx * P_prim[0] * P_prim[1] * inv_z2,
                fx + fx * P_prim[0] * P_prim[0] * inv_z2,
                -fx * P_prim[1] * inv_z,
                0, 
                fy * inv_z,
                -fy * P_prim[1] * inv_z2,
                -fy - fy * P_prim[1] * P_prim[1] * inv_z2,
                fy * P_prim[0] * P_prim[1] * inv_z2,
                fy * P_prim[0] * inv_z;
            J = -1 * J;

	    // END YOUR CODE HERE

            H += J.transpose() * J;
            b += -J.transpose() * e;

        }

	// solve dx 
        Vector6d dx;

        // START YOUR CODE HERE 
        dx = H.ldlt().solve(b);
        // END YOUR CODE HERE

        if (isnan(dx[0])) {
            cout << "result is nan!" << endl;
            break;
        }

        if (iter > 0 && cost >= lastCost) {
            // cost increase, update is not good
            cout << "cost: " << cost << ", last cost: " << lastCost << endl;
            break;
        }

        // update your estimation
        // START YOUR CODE HERE 
        T_esti = Sophus::SE3::exp(dx) * T_esti;
        // END YOUR CODE HERE
        
        lastCost = cost;

        cout << "iteration " << iter << " cost=" << cout.precision(12) << cost << " dx.norm()=" << dx.norm()  <<endl;
    }

    cout << "estimated pose: \n" << T_esti.matrix() << endl;

    return 0;
}
