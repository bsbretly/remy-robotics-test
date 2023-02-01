cmake_minimum_required(VERSION 3.0.2)
project(pick_and_place)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
set(CMAKE_CXX_STANDARD 14)

include(FindPkgConfig)

include_directories(${SYMENGINE_INCLUDE_DIR})

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        gazebo_msgs
        tf2_geometry_msgs
        moveit_ros_planning_interface
        )


find_package(Eigen3 REQUIRED)
catkin_package(
  INCLUDE_DIRS include
  ${EIGEN3_INCLUDE_DIRS}
  CATKIN_DEPENDS roscpp rospy std_msgs
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

add_executable(${PROJECT_NAME} src/pick_and_place.cpp src/pick_and_place_node.cpp)

add_dependencies(${PROJECT_NAME}
        ${catkin_EXPORTED_TARGETS}
        )

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
