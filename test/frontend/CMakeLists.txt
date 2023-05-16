cmake_minimum_required(VERSION 3.10)
project(test_frontend)
set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS "-std=c++14")

find_package(OpenCV 3.4.16 REQUIRED
    PATHS /opt/opencv3416
)
# find_package(FMT REQUIRED)

# aux_source_directory(../../src SOURCES)

include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ../../
)

add_executable(${PROJECT_NAME}
    test_frontend.cpp
    ../../src/frontend.cpp
    ../../src/config.cpp
    ../../src/frame.cpp

)
target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
# ${FMT_LIBS}
)
