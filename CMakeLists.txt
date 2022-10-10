cmake_minimum_required(VERSION 3.1.3)
project(semantic_planning_panda)

#set(CMAKE_CXX_STANDARD 14)
#set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
    cmake_modules
    roscpp
    kdl_parser
    orocos_kdl
    )

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

include_directories(
        include
    ${catkin_INCLUDE_DIRS}
    ${Eigen_INCLUDE_DIRS}
    )

add_library(examples_common STATIC
  src/examples_common.cpp
)
target_link_libraries(examples_common PUBLIC Franka::Franka ${Eigen_LIBRARIES})

add_library(my_library SHARED
        src/position_control.cpp
        src/velocity_control.cpp
        src/tube_control.cpp
        src/compliant_control.cpp
        src/data_saver.cpp
        src/semantic_mpc_controller.cpp
        )
target_link_libraries(my_library PUBLIC Franka::Franka ${Eigen_LIBRARIES})

add_library(my_controller_plugins src/rosfranka_test.cpp)
target_link_libraries(my_controller_plugins PUBLIC
        my_library
        Franka::Franka
        ${catkin_LIBRARIES}
        )

add_executable(main src/main_vereshchagin.cpp)
target_link_libraries(main PUBLIC
  Franka::Franka
  ${Eigen_LIBRARIES}
  ${catkin_LIBRARIES}
  Threads::Threads
  examples_common
  my_library
)

add_executable(main_ros src/main_rosfranka_test.cpp)
target_link_libraries(main_ros PUBLIC
        Franka::Franka
        ${Eigen_LIBRARIES}
        ${catkin_LIBRARIES}
        Threads::Threads
        examples_common
        my_library
        )
