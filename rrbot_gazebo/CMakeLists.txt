cmake_minimum_required(VERSION 2.8.3)
project(rrbot_gazebo)


add_compile_options(-std=c++11)


find_package(Eigen3 REQUIRED)

# Eigen 3.2 (Wily) only provides EIGEN3_INCLUDE_DIR, not EIGEN3_INCLUDE_DIRS
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

find_package(catkin REQUIRED
roscpp
  rospy
  std_msgs
keyboard
urdf
gazebo_ros
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_visual_tools
)


find_package(roscpp REQUIRED)
find_package(std_msgs REQUIRED)


include_directories( ${roscpp_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})

# Find Gazebo
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

#  Roboptim


#This is required to specify catkin-specific information to the build system which in turn is used to generate pkg-config and CMake files.
catkin_package(
CATKIN_DEPENDS roscpp rospy std_msgs
moveit_core
    moveit_ros_planning_interface
    interactive_markers
urdf
gazebo_ros

 DEPENDS # non-catkin CMAKE projects this one depends on
    EIGEN3

)

find_package(PkgConfig)
pkg_check_modules(Roboptim REQUIRED roboptim-core)

message("${Roboptim_LIBRARIES}******************************************************************************************************************************************************************************************************************************************************************************************************************************************")
# FIND_LIBRARY(roboptim_LIBRARIES NAMES  libroboptim-core.so.3.2.0 PATHS /usr/local/lib)


include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS} )
include_directories(include ${catkin_INCLUDE_DIRS} ${Roboptim_INCLUDE_DIRS})


link_directories(${catkin_LIBRARY_DIRS} )

add_executable(joint_publisher_node src/joint_publisher.cpp)
target_link_libraries(joint_publisher_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(sensor_reader_node src/sensor_reader.cpp)
target_link_libraries(sensor_reader_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})


add_executable(forward_kin_node src/forward_kin.cpp)
target_link_libraries(forward_kin_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})


add_executable(COM_motion_node src/COM_motion.cpp)
target_link_libraries(COM_motion_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})

link_directories(/usr/local/lib/)
add_executable(keyboard_control_node src/keyboard_control_node.cpp src/COM_computation.cpp)
target_link_libraries(keyboard_control_node ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${Roboptim_LIBRARIES} )

#add_executable(non_lin_solver src/non_lin_solver.cpp )
#target_link_libraries(non_lin_solver ${catkin_LIBRARIES} ${Boost_LIBRARIES} )



install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY include
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
