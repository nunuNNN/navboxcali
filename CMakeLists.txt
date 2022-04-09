cmake_minimum_required(VERSION 2.8.12)
project(navboxcalib)

SET(CMAKE_BUILD_TYPE "Release")


include_directories(
    ${PROJECT_SOURCE_DIR}/src/matrix
    ${PROJECT_SOURCE_DIR}/src/sins
    ${PROJECT_SOURCE_DIR}/src
)


add_executable(navBoxCalib
    ${PROJECT_SOURCE_DIR}/src/main.cpp
    ${PROJECT_SOURCE_DIR}/src/matrix/Vector3.cpp
    ${PROJECT_SOURCE_DIR}/src/matrix/Matrix3.cpp
    ${PROJECT_SOURCE_DIR}/src/matrix/Quaternion.cpp
    ${PROJECT_SOURCE_DIR}/src/sins/Earth.cpp
    ${PROJECT_SOURCE_DIR}/src/sins/IMU.cpp
    ${PROJECT_SOURCE_DIR}/src/sins/SINS.cpp
)

target_link_libraries(navBoxCalib)

