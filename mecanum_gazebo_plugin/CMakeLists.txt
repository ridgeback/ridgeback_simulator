cmake_minimum_required(VERSION 2.8.3)
project(mecanum_gazebo_plugin)

find_package(catkin REQUIRED COMPONENTS gazebo_plugins gazebo_ros)
find_package(gazebo)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mecanum_gazebo_plugin
  CATKIN_DEPENDS gazebo_plugins gazebo_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/mecanum_plugin)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)  

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
