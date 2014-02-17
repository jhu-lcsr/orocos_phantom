cmake_minimum_required(VERSION 2.8.3)

project(oro_phantom_hw)

find_package(catkin REQUIRED COMPONENTS
  tf tf_conversions rtt_ros)

find_package(Orocos-KDL REQUIRED)

catkin_package()

set(CMAKE_BUILD_TPYE Debug)

include_directories(
  ${Orocos-KDL_INCLUDE_DIRS}
  ${USE_OROCOS_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS})

link_directories(
  ${Orocos-KDL_LIBRARY_DIRS}
  ${USE_OROCOS_LIBRARY_DIRS})

orocos_component(${PROJECT_NAME}
  src/oro_phantom_hw_components.cpp
  src/phantom_omni.h
  src/phantom_omni.cpp)

target_link_libraries(${PROJECT_NAME}
  ${Orocos-KDL_LIBRARIES}
  ${USE_OROCOS_LIBRARIES}
  ${catkin_LIBRARIES}
  HD HDU rt)

orocos_generate_package()
