cmake_minimum_required(VERSION 2.8.3)

project(oro_phantom_hw)

find_package(catkin REQUIRED COMPONENTS
  tf tf_conversions rtt_ros rtt_ros_tools rtt_rosclock)

find_package(orocos_kdl REQUIRED)

#orocos_use_package(kdl_typekit)

catkin_package()

set(CMAKE_BUILD_TPYE Debug)

include_directories(
  ${orocos_kdl_INCLUDE_DIRS}
  ${USE_OROCOS_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS})

link_directories(
  ${orocos_kdl_LIBRARY_DIRS}
  ${USE_OROCOS_LIBRARY_DIRS})

orocos_component(${PROJECT_NAME}
  src/oro_phantom_hw_components.cpp
  src/phantom_omni.h
  src/phantom_omni.cpp)

target_link_libraries(${PROJECT_NAME}
  ${orocos_kdl_LIBRARIES}
  ${USE_OROCOS_LIBRARIES}
  ${catkin_LIBRARIES}
  HD HDU rt)

orocos_generate_package()
