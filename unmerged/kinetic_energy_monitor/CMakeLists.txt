cmake_minimum_required(VERSION 2.8.3)
project(kinetic_energy_monitor)
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

find_package(PCL REQUIRED)
find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    geometry_msgs
    nav_msgs
    std_msgs
    roslint
    tf
    message_generation
)

include_directories(
  ros/include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_service_files(
  DIRECTORY srv
  FILES
  KineticEnergyMonitorMsg.srv
)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(
  INCLUDE_DIRS ros/include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    geometry_msgs
    nav_msgs
    std_msgs
    roslint
    tf
)

add_library(${PROJECT_NAME}
  ros/src/kinetic_energy_monitor_node.cpp
  ros/src/kinetic_energy_monitor.cpp
)

add_executable(kinetic_energy_monitor_server ros/src/kinetic_energy_monitor_node.cpp ros/src/ ros/src/kinetic_energy_monitor.cpp)
add_dependencies(kinetic_energy_monitor_server kinetic_energy_monitor_gencpp)
target_link_libraries(kinetic_energy_monitor_server ${catkin_LIBRARIES} ${PCL_LIBRARIES})


install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY ros/include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE)
