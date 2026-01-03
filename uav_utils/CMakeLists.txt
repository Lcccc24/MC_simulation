cmake_minimum_required(VERSION 3.0.2)
project(uav_utils)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  gazebo_msgs
  geometry_msgs
  mavros_msgs
  roscpp
  quadrotor_msgs
  tf2_ros
)

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES uav_utils
 CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${CATKIN_DEVEL_PREFIX}/include

)


# add_executable(${PROJECT_NAME}_node src/uav_utils_node.cpp)
add_executable(apriltag_pose_ctrl src/apriltag_pose_ctrl.cpp)
add_executable(offboard_node src/offboard_node.cpp)
add_executable(uav0_onboard_msg src/uav0_onboard_msg.cpp)
add_executable(uav_pose_vis src/uav_pose_vis.cpp)
add_executable(uav_landing_pose_raw src/uav_landing_pose_raw.cpp)
add_executable(gazebo_true src/gazebo_true.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(apriltag_pose_ctrl ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(offboard_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(uav0_onboard_msg ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(uav_pose_vis ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(uav_landing_pose_raw ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(gazebo_true ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )
target_link_libraries(apriltag_pose_ctrl
  ${catkin_LIBRARIES}
)
target_link_libraries(offboard_node
  ${catkin_LIBRARIES}
)
target_link_libraries(uav0_onboard_msg
  ${catkin_LIBRARIES}
)
target_link_libraries(uav_pose_vis
  ${catkin_LIBRARIES}
)
target_link_libraries(uav_landing_pose_raw
  ${catkin_LIBRARIES}
)
target_link_libraries(gazebo_true
  ${catkin_LIBRARIES}
)


install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)



## Add gtest based cpp test target and link libraries
catkin_add_gtest(${PROJECT_NAME}-test src/${PROJECT_NAME}_test.cpp)
if(TARGET ${PROJECT_NAME}-test)
  target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)



