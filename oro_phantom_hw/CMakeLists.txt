cmake_minimum_required(VERSION 2.8.3)

project(oro_phantom_hw)

find_package(catkin REQUIRED COMPONENTS
  tf tf_conversions rtt_ros_tools rtt_telemanip_msgs)

find_package(OROCOS-RTT REQUIRED)
include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)

catkin_package(DEPENDS rtt ocl)

include_directories(
  ${USE_OROCOS_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS})

link_directories(${catkin_LIBRARY_DIRS})

orocos_component(${PROJECT_NAME}
  src/oro_phantom_hw_components.cpp
  src/phantom_omni.h
  src/phantom_omni.cpp)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${USE_OROCOS_LIBRARIES}
  HD HDU rt)

orocos_generate_package()
