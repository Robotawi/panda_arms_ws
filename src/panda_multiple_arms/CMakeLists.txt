cmake_minimum_required(VERSION 3.0.2)
project(panda_multiple_arms)

find_package(catkin REQUIRED COMPONENTS
    roscpp

    # for robot arm pose
    geometry_msgs
    
    # for moveit planning
    moveit_ros_planning_interface
    moveit_ros_planning
)


## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES panda_multiple_arms
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)


## Specify additional locations of header files
include_directories(
# include
${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(plan_simple_motion src/plan_simple_motion.cpp)


## Add cmake target dependencies of the executable
add_dependencies(plan_simple_motion ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(plan_simple_motion
  ${catkin_LIBRARIES}
)

