#include <iostream>
#include "hello.h"
#include <gflags/gflags.h>

DEFINE_int32(print_times, 1, "how many times it prints");

int main( int argc, char** argv ) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  std::cout << "print_times: " << FLAGS_print_times << std::endl; 
 for(int i=0; i < FLAGS_print_times; ++i)
 {
  sayHello();
 } 
  return 0;
}
