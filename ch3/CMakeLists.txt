cmake_minimum_required(VERSION 3.16)
project(traject)
find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)
find_package(Pangolin REQUIRED)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR})
include_directories(
	${Sophus_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIRS}
	${Pangolin_INCLUDE_DIRS}
)
add_executable(draw_traj draw_trajectory.cpp)
target_link_libraries(draw_traj ${Sophus_LIBRARIES} ${EIGEN3_LIBS} ${Pangolin_LIBRARIES})

