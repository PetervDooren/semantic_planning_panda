cmake_minimum_required(VERSION 3.1.3)
project(flexcraft_panda)

#set(CMAKE_CXX_STANDARD 14)
#set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED)
find_package(Franka REQUIRED)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

add_library(examples_common STATIC
  src/examples_common.cpp
)
target_link_libraries(examples_common PUBLIC Franka::Franka Eigen3::Eigen)

add_executable(run_test src/main.cpp)
target_link_libraries(run_test PUBLIC
  Franka::Franka
  Eigen3::Eigen
  Threads::Threads
  examples_common
)
