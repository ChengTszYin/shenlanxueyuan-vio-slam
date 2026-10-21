cmake_minimum_required(VERSION 3.16)
project(directBA)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR})
set(g2o_DIR "/usr/local/lib/cmake/g2o")
find_package(g2o REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Sophus REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS} ${G2O_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} ${Sophus_INCLUDE_DIRS} ${Pangolin_INCLUDE_DIRS})
add_library(bal_common common.cpp)
add_executable(directBA directBA.cpp)
add_executable(bal_data bal_data.cpp)

target_include_directories(directBA PRIVATE
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${Pangolin_INCLUDE_DIRS}
)

target_link_libraries(directBA
  ${OpenCV_LIBS}
  ${Pangolin_LIBRARIES}
  g2o::core
  g2o::stuff
  g2o::types_sba
  Eigen3::Eigen
  GL
  GLEW
  /usr/local/lib/libSophus.so
)

target_include_directories(bal_data PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${EIGEN3_INCLUDE_DIR}
)

find_library(CSPARSE_LIBRARY cxsparse)

target_link_libraries(bal_data
  bal_common
  g2o::core
  g2o::stuff
  g2o::solver_csparse
  g2o::csparse_extension
  cxsparse
  Eigen3::Eigen
  /usr/local/lib/libSophus.so
)
