cmake_minimum_required(VERSION 2.8.3)
project(bebop_tools)

find_package(catkin REQUIRED COMPONENTS
  bebop_msgs roscpp
)

# catkin_python_setup()

catkin_package(
  #  INCLUDE_DIRS include
 CATKIN_DEPENDS roscpp bebop_msgs
)

add_executable(keyboard_node src/keyboard.cpp)
target_link_libraries(keyboard_node ${catkin_LIBRARIES})


# TODO: Install Rules
