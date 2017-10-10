cmake_minimum_required(VERSION 2.8.3)
project(footprint_checker)
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

find_package(PCL REQUIRED)
find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    geometry_msgs
    nav_msgs
    map_msgs
    std_msgs
    sensor_msgs
    roslint
    costmap_2d
    tf
    message_generation
)

include_directories(
  ros/include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_service_files(
  FILES
  CollisionCheckerMsg.srv
)

generate_messages(DEPENDENCIES std_msgs sensor_msgs geometry_msgs)

catkin_package(
  INCLUDE_DIRS ros/include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    geometry_msgs
    nav_msgs
    map_msgs
    std_msgs
    sensor_msgs
    roslint
    costmap_2d
    tf
)

add_library(${PROJECT_NAME}
  ros/src/base_collision_checker.cpp
  ros/src/collision_checker.cpp
  ros/src/base_collision_checker_node.cpp
)

add_executable(base_collision_checker_server ros/src/base_collision_checker_node.cpp ros/src/base_collision_checker.cpp ros/src/collision_checker.cpp )
target_link_libraries(base_collision_checker_server ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(base_collision_checker_server footprint_checker_gencpp)


install(TARGETS ${PROJECT_NAME} footprint_checker
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY ros/include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE)
