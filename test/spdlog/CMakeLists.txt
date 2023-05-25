cmake_minimum_required(VERSION 3.10)
project(test_spdlog)
set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS "-std=c++14")

add_executable(${PROJECT_NAME}
    test_spdlog.cpp
)

target_link_libraries(${PROJECT_NAME}
)