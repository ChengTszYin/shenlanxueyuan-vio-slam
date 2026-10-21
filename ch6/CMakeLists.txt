cmake_minimum_required(VERSION 3.16)
project(flow)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR})

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)
find_package(Pangolin REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} ${Sophus_INCLUDE_DIRS} ${Pangolin_INCLUDE_DIRS})

add_executable(optical_flow optical_flow.cpp)

add_executable(direct_method direct_method.cpp)

add_executable(disparity disparity.cpp)

target_link_libraries(optical_flow ${OpenCV_LIBS} ${EIGEN3_LIBS})

target_link_libraries(direct_method ${OpenCV_LIBS} ${EIGEN3_LIBS} ${Sophus_LIBRARIES} ${Pangolin_LIBRARIES})

target_link_libraries(disparity ${OpenCV_LIBS} ${EIGEN3_LIBS} ${Sophus_LIBRARIES} ${Pangolin_LIBRARIES})