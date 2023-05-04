cmake_minimum_required(VERSION 3.10)
project(orb_cv)
SET(CMAKE_BUILD_TYPE "Debug")

find_package(OpenCV 3.4.16 REQUIRED
    PATHS /opt/opencv3416
)

include_directories(
    ${OpenCV_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}
    src/orb_cv.cpp
)
target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
)

