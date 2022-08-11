cmake_minimum_required(VERSION 3.1.3)
project(semantic_planning_panda)

#set(CMAKE_CXX_STANDARD 14)
#set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS cmake_modules)

find_package(Eigen REQUIRED)
find_package(Franka REQUIRED)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

catkin_package(
#    INCLUDE_DIRS ...
#    LIBRARIES ...
#    CATKIN DEPENDS ...
    DEPENDS Eigen
    )

include_directories( ${Eigen_INCLUDE_DIRS} )
add_library(examples_common STATIC
  src/examples_common.cpp
)
target_link_libraries(examples_common PUBLIC Franka::Franka ${Eigen_LIBRARIES})

add_library(my_library STATIC
  src/position_control.cpp
  src/velocity_control.cpp
  src/tube_control.cpp
  src/compliant_control.cpp
)
target_link_libraries(my_library PUBLIC Franka::Franka ${Eigen_LIBRARIES})

#add_executable(run_test src/main.cpp src/semantic_mpc_controller.cpp)
#target_link_libraries(run_test PUBLIC
#  Franka::Franka
#  ${Eigen_LIBRARIES}
#  Threads::Threads
#  examples_common
#)

add_executable(topo_planner src/main_topo_planner.cpp)
target_link_libraries(topo_planner PUBLIC
  Franka::Franka
  ${Eigen_LIBRARIES}
  Threads::Threads
  examples_common
  my_library
)
