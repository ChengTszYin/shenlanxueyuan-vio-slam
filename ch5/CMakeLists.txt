cmake_minimum_required(VERSION 3.15)
project(vio1)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR})

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)
find_package(TBB REQUIRED)
find_package(Pangolin REQUIRED)


include_directories(${OPENCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} ${Sophus_INCLUDE_DIRS} ${Pangolin_INCLUDE_DIRS})

add_executable(computeORB computeORB.cpp)
add_executable(E2Rt E2Rt.cpp)
add_executable(GN_BA GN_BA.cpp)
add_executable(icp icp.cpp)

target_link_libraries(computeORB 
                    ${OpenCV_LIBS}
                    TBB::tbb
                    )
target_link_libraries(E2Rt ${EIGEN3_LIBS} ${Sophus_LIBRARIES})
target_link_libraries(GN_BA ${EIGEN3_LIBS} ${Sophus_LIBRARIES} ${OpenCV_LIBS} ${Pangolin_LIBRARIES})
target_link_libraries(icp ${Pangolin_LIBRARIES} ${EIGEN3_LIBS} ${Sophus_LIBRARIES})
