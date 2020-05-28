cmake_minimum_required(VERSION 2.8.3)
project(roscopter)

set(CMAKE_CXX_STANDARD 17)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  dynamic_reconfigure
  rosflight_msgs
  rosflight_utils
  roscopter_utils
  std_msgs
  nav_msgs
  sensor_msgs
  rosbag
  roslib
)

set(EIGEN3_INCLUDE_DIR "/usr/local/include/eigen3")
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})
set(GCC_COVERAGE_LINK_FLAGS    "-g")
set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} ${GCC_COVERAGE_LINK_FLAGS}")

list(APPEND CMAKE_PREFIX_PATH /opt/drake-latest-bionic/drake)

find_package(drake REQUIRED)
find_package(Eigen3 3.3.9 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(inertial_sense QUIET)


#include_directories(${CMAKE_SOURCE_DIR}/rosflight/rosflight_utils/include)

set(catkin_LIBRARIES ${catkin_LIBRARIES} drake::drake)

if (${inertial_sense_FOUND})
    message(WARNING "found inertial sense library")
    add_definitions(-DINERTIAL_SENSE)
else()
    message(WARNING "Did NOT find inertial sense library")
endif()

find_package(ublox QUIET)

if (${ublox_FOUND})
    message(WARNING "found ublox library")
    add_definitions(-DUBLOX)
else()
    message(WARNING "Did NOT find ublox library")
endif()

generate_dynamic_reconfigure_options(
  cfg/Controller.cfg
)

catkin_package(
  INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIRS}
  CATKIN_DEPENDS roscpp rospy dynamic_reconfigure
        eigen_conversions
    rosflight_msgs rosflight_utils std_msgs nav_msgs sensor_msgs roslib roscopter_utils
  DEPENDS EIGEN3
)

add_definitions(-DROSCOPTER_DIR="${CMAKE_CURRENT_LIST_DIR}")

###########
## Build ##
###########

include_directories(include 
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  )
message (STATUS "catkin include dirs" ${catkin_INCLUDE_DIRS})

## Declare cpp executables
add_executable(controller
  src/controller/controller_node.cpp
  src/controller/controller.cpp
  include/controller/controller.h
  src/controller/simple_pid.cpp)
add_dependencies(controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(controller ${catkin_LIBRARIES})

add_library(ekf
  src/ekf/state.cpp
  src/ekf/dynamics.cpp
  src/ekf/ekf.cpp
  src/ekf/meas.cpp
  src/ekf/log.cpp
)
target_include_directories(ekf PUBLIC include ${catkin_INCLUDE_DIRS})
target_link_libraries(ekf ${YAML_CPP_LIBRARIES} stdc++fs)

add_library(ekf_ros src/ekf/ekf_ros.cpp)
target_link_libraries(ekf_ros ekf ${catkin_LIBRARIES})
add_dependencies(ekf_ros ${catkin_EXPORTED_TARGETS})

add_executable(ekf_node src/ekf/ekf_node.cpp)
target_link_libraries(ekf_node ekf_ros ${catkin_LIBRARIES})
add_dependencies(ekf_node ${catkin_EXPORTED_TARGETS})

add_executable(ekf_rosbag src/ekf/ekf_rosbag.cpp)
target_link_libraries(ekf_rosbag ekf_ros ${catkin_LIBRARIES})
add_dependencies(ekf_rosbag ${catkin_EXPORTED_TARGETS})

#add_executable(publish_arm scripts/publish_arm.cpp scripts/PID_controller.cpp scripts/PID_controller.h scripts/CustomAttitudeController.cpp scripts/CustomAttitudeController.h)
#target_link_libraries(publish_arm  ${catkin_LIBRARIES})
#add_dependencies(ekf_rosbag ${catkin_EXPORTED_TARGETS})

#add_executable(trajectory_optimization scripts/TrajectoryOptimization.cpp scripts/TrajectoryOptimization.h scripts/Drone.h scripts/Drone.cpp scripts/constants.h)
#target_link_libraries(trajectory_optimization  ${catkin_LIBRARIES})
#add_dependencies(trajectory_optimization ${catkin_EXPORTED_TARGETS})

message("Eigen version = " ${Eigen3_VERSION})

add_executable(customAttitudeController
        scripts/CustomAttitudeController.cpp
        scripts/PID_controller.cpp scripts/PID_controller.h
        scripts/CustomAttitudeController.cpp scripts/CustomAttitudeController.h
        scripts/TrajectoryOptimization.cpp scripts/TrajectoryOptimization.h
        scripts/Drone.h scripts/Drone.cpp scripts/constants.h
        scripts/configParser.h scripts/configParser.cpp
        include/roscopter/quaternionMath.h
        include/roscopter/quaternion.h)
target_link_libraries(customAttitudeController ${catkin_LIBRARIES} libpystring.a)
add_dependencies(customAttitudeController ${catkin_EXPORTED_TARGETS})