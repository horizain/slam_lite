cmake_minimum_required(VERSION 3.10)
project(test_triangulation)
set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS "-std=c++14")


find_package(OpenCV 3.4.16 REQUIRED
    PATHS /opt/opencv3416
)
find_package(fmt REQUIRED)

# gtest
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})

include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ../../
)

add_executable(${PROJECT_NAME}
    test_triangulation.cpp
)

target_link_libraries(${PROJECT_NAME}
fmt::fmt
pthread
${GTEST_BOTH_LIBRARIES}
)