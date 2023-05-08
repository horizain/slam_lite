cmake_minimum_required(VERSION 3.10)
project(orb_lite)
SET(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
find_package(OpenCV 3.4.16 REQUIRED
    PATHS /opt/opencv3416
)

include_directories(
    ${OpenCV_INCLUDE_DIRS}
    include/
)

add_executable(${PROJECT_NAME}
    src/FASTDetector.cpp
    src/orb_self.cpp

)
target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
)

add_executable(test
    src/test.cpp

)
target_link_libraries(test
${OpenCV_LIBS}
)

