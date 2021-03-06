cmake_minimum_required(VERSION 2.8.12)
project(navboxcalib)

#使用release编译存在段错误
SET(CMAKE_BUILD_TYPE "release")
# SET(CMAKE_BUILD_TYPE "Debug")
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g2 -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

include_directories(
    ${PROJECT_SOURCE_DIR}/src/matrix
    ${PROJECT_SOURCE_DIR}/src/sins
    ${PROJECT_SOURCE_DIR}/src/kalman
    ${PROJECT_SOURCE_DIR}/src/IOfile
    ${PROJECT_SOURCE_DIR}/src
)


add_executable(navBoxCalib
    ${PROJECT_SOURCE_DIR}/src/main.cpp

    ${PROJECT_SOURCE_DIR}/src/matrix/Vector3.cpp
    ${PROJECT_SOURCE_DIR}/src/matrix/Matrix3.cpp
    ${PROJECT_SOURCE_DIR}/src/matrix/Quaternion.cpp
    ${PROJECT_SOURCE_DIR}/src/matrix/Matrix.cpp

    ${PROJECT_SOURCE_DIR}/src/sins/Earth.cpp
    ${PROJECT_SOURCE_DIR}/src/sins/IMU.cpp
    ${PROJECT_SOURCE_DIR}/src/sins/SINS.cpp

    ${PROJECT_SOURCE_DIR}/src/kalman/AVPInterp.cpp
    ${PROJECT_SOURCE_DIR}/src/kalman/Kalman.cpp
    ${PROJECT_SOURCE_DIR}/src/kalman/SGClbrt.cpp
    ${PROJECT_SOURCE_DIR}/src/kalman/SGOCalbrt.cpp

    ${PROJECT_SOURCE_DIR}/src/IOfile/IOfile.cpp
)

target_link_libraries(navBoxCalib)

