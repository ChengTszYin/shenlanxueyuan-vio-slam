#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>
using namespace std;
#define MATRIX_SIZE 100

int main(int argc, char** argv)
{
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> alpha;
  alpha = Eigen::MatrixXd::Random(MATRIX_SIZE, MATRIX_SIZE);
  alpha = alpha * alpha.transpose(); 
  Eigen::Matrix<double, MATRIX_SIZE, 1> beta = Eigen::MatrixXd::Random(MATRIX_SIZE, 1);
  //cout << "Matrix A is \n" << alpha.transpose() << endl;
  //cout << endl;
  //cout << "B is \n" << beta.transpose() << endl;
  //cout << endl; 
  Eigen::Matrix<double, MATRIX_SIZE, 1> x = alpha.colPivHouseholderQr().solve(beta);
  cout << "QR decomposition result is \n" << x.transpose() << endl;
  cout << endl;
  x = alpha.ldlt().solve(beta);
  cout << "cholesky decomposition result is \n" << x.transpose() << endl;

  return 0;
}
