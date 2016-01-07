cmake_minimum_required(VERSION 2.8.3)
project(pointcloud_assembler_trigger)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  dynamic_reconfigure
  geometry_msgs
  laser_assembler
  sensor_msgs
)

generate_dynamic_reconfigure_options(
  cfg/AssemblerTrigger.cfg
)

#catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES assembler_trigger
#  CATKIN_DEPENDS dynamic_reconfigure geometry_msgs laser_assembler sensor_msgs
#)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(trigger_interface_provider src/TriggerInterfaceProvider.cpp)
add_library(assembler_trigger src/AssemblerTrigger.cpp)

add_executable(assembler_trigger_node src/AssemblerTriggerNode.cpp)
add_dependencies(assembler_trigger ${PROJECT_NAME}_gencfg)

target_link_libraries(assembler_trigger
  trigger_interface_provider
  ${catkin_LIBRARIES})

target_link_libraries(assembler_trigger_node
  assembler_trigger
  ${catkin_LIBRARIES})

# TODO(fkunz): deprecated, delete after restructuring
add_executable(assembler_trigger_old src/pc_assembler_trigger.cpp)
#add_dependencies(assembler_trigger_old  ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(assembler_trigger_old
  ${catkin_LIBRARIES}
)
