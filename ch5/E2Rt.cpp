//
// Created by 高翔 on 2017/12/19.
// 本程序演示如何从Essential矩阵计算R,t
//

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace Eigen;

#include <sophus/so3.h>
#include <cmath>
#include <iostream>

using namespace std;

int main(int argc, char **argv) {

    // 给定Essential矩阵
    Matrix3d E;
    E << -0.0203618550523477, -0.4007110038118445, -0.03324074249824097,
            0.3939270778216369, -0.03506401846698079, 0.5857110303721015,
            -0.006788487241438284, -0.5815434272915686, -0.01438258684486258;

    // 待计算的R,t
    Matrix3d R;
    Vector3d t;

    // SVD and fix sigular values
    // START YOUR CODE HERE
    Matrix3d R_minus;
    R = AngleAxisd(M_PI / 2, Vector3d(0, 0, 1)).toRotationMatrix();
    Sophus::SO3 SO3_R(R);

    R_minus = AngleAxisd(-M_PI / 2, Vector3d(0, 0, 1)).toRotationMatrix();
    Sophus::SO3 SO3_R_minus(R_minus);

    JacobiSVD<Matrix3d> svd(E, ComputeFullU | ComputeFullV);
    Matrix3d U = svd.matrixU();
    Matrix3d V = svd.matrixV();
    Vector3d sigma = svd.singularValues();
    double d = (sigma[0] + sigma[1]) / 2;
    Matrix3d sum = Matrix3d::Zero();
    sum(0, 0) = d;
    sum(1, 1) = d;

    // END YOUR CODE HERE

    // set t1, t2, R1, R2 
    // START YOUR CODE HERE
    Matrix3d t_wedge1;
    Matrix3d t_wedge2;

    Matrix3d R1;
    Matrix3d R2;

    t_wedge1 = U * SO3_R.matrix() * sum * U.transpose();
    t_wedge2 = U * SO3_R_minus.matrix() * sum * U.transpose();
    R1 = U * SO3_R.matrix() * V.transpose();
    R2 = U * SO3_R_minus.matrix() * V.transpose();
    // END YOUR CODE HERE

    cout << "R1 = " << R1 << endl;
    cout << "R2 = " << R2 << endl;
    cout << "t1 = " << Sophus::SO3::vee(t_wedge1) << endl;
    cout << "t2 = " << Sophus::SO3::vee(t_wedge2) << endl;

    // check t^R=E up to scale
    Matrix3d tR = t_wedge1 * R1;
    cout << "t^R = " << tR << endl;

    return 0;
}