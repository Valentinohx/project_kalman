cmake_minimum_required(VERSION 2.8)
project(gazebo_sim_control)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
)

find_package(gazebo REQUIRED)
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

catkin_package(
  CATKIN_DEPENDS
    roscpp
    std_msgs
    geometry_msgs
    tf2
    tf2_ros
    gazebo_ros
)


include_directories(${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

# plugin to trigger simulation steps via a ROS topic
add_library(gzsim_control_step SHARED src/simulation_step.cpp)
target_link_libraries(gzsim_control_step ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

# TODO: complete the tf publisher plugin.
# plugin to publish transformation information into /tf
add_library(gzsim_control_tfpub SHARED src/tf_publisher.cpp)
target_link_libraries(gzsim_control_tfpub ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} -Wl,--no-undefined)

install(
  TARGETS
    gzsim_control_step
    gzsim_control_tfpub
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
