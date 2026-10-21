cmake_minimum_required(VERSION 2.8)
project(imageVision)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR})

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

add_executable(undistort_image undistort_image.cpp)
add_executable(fisheye fisheye.cpp)
add_executable(disparity disparity.cpp)
add_executable(gaussnewton gaussnewton.cpp)

target_link_libraries(undistort_image ${OpenCV_LIBS})
target_link_libraries(fisheye ${OpenCV_LIBS})
target_link_libraries(disparity ${OpenCV_LIBS} ${EIGN3_LIBS} ${Pangolin_LIBRARIES})
target_link_libraries(gaussnewton ${OpenCV_LIBS} ${EIGN3_LIBS})