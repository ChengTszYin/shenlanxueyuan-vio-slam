cmake_minimum_required(VERSION 3.16)
PROJECT(helloslam)
SET(CMAKE_BUILD_TYPE Release)

#=====use PkgConfig to search dependencies=====
FIND_PACKAGE(PkgConfig REQUIRED)
pkg_check_modules(GFLAGS REQUIRED gflags)
pkg_check_modules(GLOG REQUIRED libglog)
pkg_check_modules(GTEST REQUIRED gtest)
#=====end=====


SET(CMAKE_INSTALL_PREFIX "/usr/local")
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include
			${GFLAGS_INCLUDE_DIR}
			${GLOG_INCLUDE_DIR}
			${GTEST_INCLUDE_DIRS}
)
ADD_LIBRARY(hello SHARED src/hello.cpp)
TARGET_LINK_LIBRARIES(hello ${GLOG_LIBRARIES})
ADD_EXECUTABLE(sayhello useHello.cpp)
TARGET_LINK_LIBRARIES(sayhello hello ${GLOG_LIBRARIES} ${GFLAGS_LIBRARIES}
)
ADD_EXECUTABLE(gtest_sayhello gtest_hello.cpp)
TARGET_LINK_LIBRARIES(gtest_sayhello hello ${GTEST_LIBRARIES} ${GTEST_MAIN_LIBRARIES} pthread)

INSTALL(TARGETS hello
	LIBRARY DESTINATION lib
)

INSTALL(TARGETS sayhello
	RUNTIME DESTINATION bin
)

INSTALL(FILES include/hello.h
	DESTINATION include
)
ENABLE_TESTING()
ADD_TEST(NAME HelloGTest COMMAND gtest_sayhello)
