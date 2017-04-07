cmake_minimum_required(VERSION 2.8.3)

project(kuka_kr10_support)
find_package(catkin REQUIRED
  COMPONENTS
    moveit_visual_tools
    rviz_visual_tools
    eigen_conversions
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    pluginlib
    tf_conversions
    cmake_modules
    geometric_shapes
    roscpp
    graph_msgs
    std_msgs
    trajectory_msgs
    cmake_modules	
)

find_package(Boost REQUIRED system filesystem date_time thread)
find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})
catkin_package(
  CATKIN_DEPENDS
    moveit_core
    rviz_visual_tools
    tf_conversions
    eigen_conversions
    moveit_ros_planning_interface
    interactive_markers
    moveit_ros_planning
    moveit_ros_planning_interface
    pluginlib
    geometric_shapes
    moveit_ros_robot_interaction
    graph_msgs
    std_msgs
    moveit_msgs
    moveit_visual_tools
)
find_package(Eigen REQUIRED)

add_definitions(-DQT_NO_KEYWORDS)

if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(test/roslaunch_test_kr10r1100sixx.xml)
endif()

install(DIRECTORY config launch meshes urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

qt4_wrap_cpp(MOC_FILES 
	src/framefab_panel.h
)

set(SOURCE_FILES src/framefab_panel.cpp ${MOC_FILES})
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
add_library(${PROJECT_NAME} ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES})
add_executable(kuka_node src/kuka_node.cpp)
target_link_libraries(kuka_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS kuka_node DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
