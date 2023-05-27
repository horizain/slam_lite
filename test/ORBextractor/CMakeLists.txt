cmake_minimum_required(VERSION 3.10)
project(test_orbextractor)
set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(OpenCV 3.4.16 REQUIRED
    PATHS /opt/opencv3416
)
# # find_package(fmt REQUIRED)
# find_package(Sophus REQUIRED)
# include_directories(${Sophus_INCLUDE_DIRS})
include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ../../include/
)

add_executable(${PROJECT_NAME}
    test_orbextractor.cpp
    ../../src/ORBextractor.cpp
)
target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
# ${Sophus_LIBRARIES}
)
