cmake_minimum_required(VERSION 2.8.3)
project(pure_pursuit_planner)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  geometry_msgs
  roscpp
  usv_navigation_msgs
  message_filters
  quaternion_operation
)

find_package(Eigen3 REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/PurePursuitPlanner.cfg
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES pure_pursuit_planner
  CATKIN_DEPENDS dynamic_reconfigure geometry_msgs roscpp usv_navigation_msgs message_filters quaternion_operation
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(pure_pursuit_planner_node src/pure_pursuit_planner_node.cpp src/pure_pursuit_planner.cpp)
target_link_libraries(pure_pursuit_planner_node ${catkin_LIBRARIES})
add_dependencies(pure_pursuit_planner_node ${catkin_EXPORTED_TARGETS})

# Install header files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS pure_pursuit_planner_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

foreach(dir launch cfg)
  install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)