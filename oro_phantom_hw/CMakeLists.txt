cmake_minimum_required(VERSION 2.8.3)

project(oro_phantom_hw)

find_package(catkin REQUIRED COMPONENTS
  tf tf_conversions rtt_ros)

catkin_package()

set(CMAKE_BUILD_TPYE Debug)

include_directories(
  ${USE_OROCOS_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS})

orocos_component(${PROJECT_NAME}
  src/oro_phantom_hw_components.cpp
  src/phantom_omni.h
  src/phantom_omni.cpp)

target_link_libraries(${PROJECT_NAME}
  ${USE_OROCOS_LIBRARIES}
  ${catkin_LIBRARIES}
  HD HDU rt)

orocos_generate_package()
