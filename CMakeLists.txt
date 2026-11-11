cmake_minimum_required(VERSION 3.20)
project(fieldpose LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(FetchContent)

FetchContent_Declare(
  eigen
  GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
  GIT_TAG 3.4.0
  GIT_SHALLOW TRUE)
set(EIGEN_BUILD_DOC OFF CACHE BOOL "" FORCE)
set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
set(EIGEN_BUILD_PKGCONFIG OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(eigen)

add_library(fieldpose
  src/kalman_filter.cpp
  src/trajectory.cpp
  src/log_loader.cpp
  src/simulator.cpp
  src/pose_fusion.cpp
  src/evaluation.cpp)
target_include_directories(fieldpose PUBLIC include)
target_link_libraries(fieldpose PUBLIC Eigen3::Eigen)
target_compile_options(fieldpose PRIVATE -Wall -Wextra -Wpedantic)

add_executable(extract_trajectories apps/extract_trajectories.cpp)
target_link_libraries(extract_trajectories PRIVATE fieldpose)

option(FIELDPOSE_BUILD_TESTS "Build unit tests" ON)
if(FIELDPOSE_BUILD_TESTS)
  FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.15.2
    GIT_SHALLOW TRUE)
  set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(googletest)
  enable_testing()
  add_executable(fieldpose_tests
    tests/test_kalman_filter.cpp
    tests/test_log_loader.cpp
    tests/test_simulator.cpp
    tests/test_pose_fusion.cpp)
  target_link_libraries(fieldpose_tests PRIVATE fieldpose GTest::gtest_main)
  include(GoogleTest)
  gtest_discover_tests(fieldpose_tests)
endif()
