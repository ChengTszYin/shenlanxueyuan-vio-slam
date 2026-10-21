cmake_minimum_required(VERSION 3.16)
project(useEign)
set(CMAKE_BUILD_TYPE Release)
find_package(Eigen3 REQUIRED)
include_directories(
${EIGEN3_INCLUDE_DIR}
)
add_executable(useEign useEign.cpp)
target_link_libraries(useEign ${EIGEN3_LIBS})
add_executable(tf tf.cpp)
target_link_libraries(tf ${EIGEN3_LIBS})
