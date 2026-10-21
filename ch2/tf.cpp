#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>
using namespace Eigen;
int main(int argc, char** argv)
{
  Quaterniond qwr(0.55, 0.3, 0.2, 0.2), qrb(0.99, 0, 0, 0.01), qbl(0.3, 0.5, 0, 20.1), qbc(0.8, 0.2, 0.1, 0.1);
  qwr.normalize();
  qrb.normalize();
  qbl.normalize();
  qbc.normalize();
  Vector3d twr(0.1, 0.2, 0.3), trb(0.05, 0, 0.5), tbl(0.4, 0, 0.5), tbc(0.5, 0.1, 0.5);
  
  Isometry3d pwr(qwr), prb(qrb), pbl(qbl), pbc(qbc);
  pwr.pretranslate(twr);
  prb.pretranslate(trb);
  pbl.pretranslate(tbl);
  pbc.pretranslate(tbc);

  Vector3d c_point(0.3, 0.2, 1.2);
  Vector3d laser_f = pbl.inverse() * pbc * c_point;
  std::cout << "这个点在激光系下的坐标: \n" <<laser_f << std::endl;
  
 Vector3d word_f = pwr * prb * pbc * c_point; 
 std::cout  << "这个点在世界系下的坐标。\n" << word_f << std::endl;
  return 0;
}
