cmake_minimum_required(VERSION 3.0.2)
project(aerial_robot_pcl)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  nodelet
  roscpp
  pcl_ros
  pcl_msgs
  jsk_topic_tools
  dynamic_reconfigure
  jsk_recognition_msgs
  nav_msgs
  message_filters
  jsk_recognition_utils
  jsk_pcl_ros
  eigen_conversions
  )

find_package(PCL REQUIRED)
find_package(Boost REQUIRED)
message("PCL ${PCL_VERSION} FOUND")

generate_dynamic_reconfigure_options(
  cfg/GroundPlaneDetection.cfg)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS nodelet roscpp pcl_ros pcl_msgs jsk_topic_tools dynamic_reconfigure jsk_recognition_msgs nav_msgs message_filters jsk_recognition_utils jsk_pcl_ros eigen_conversions
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  )

# download and install sample data
#add_custom_target(install_sample_data ALL COMMAND ${PROJECT_SOURCE_DIR}/scripts/install_test_data.py)

macro(arp_add_nodelet _nodelet_cpp _nodelet_class _single_nodelet_exec_name)
  jsk_nodelet(${_nodelet_cpp} ${_nodelet_class} ${_single_nodelet_exec_name}
    ${PROJECT_NAME}_nodelet_sources ${PROJECT_NAME}_nodelet_executables)
endmacro()

arp_add_nodelet(src/ground_plane_detection.cpp "aerial_robot_pcl/groundPlaneDetection" "ground_plane_detection")

add_library(${PROJECT_NAME} SHARED ${aerial_robot_pcl_nodelet_sources})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)


if(catkin_EXPORTED_LIBRARIES)
  add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_LIBRARIES})
endif()

# test
# if(CATKIN_ENABLE_TESTING)
#   find_package(catkin REQUIRED COMPONENTS rostest roslaunch)
#   if(roslaunch_VERSION VERSION_LESS "1.11.1")
#     message(WARNING "roslaunch_add_file check fails with unsupported doc attributes ${roslaunch_VERSION}")
#   else()
#     file(GLOB LAUNCH_FILES launch/*.launch)
#     foreach(LAUNCH_FILE ${LAUNCH_FILES})
#       roslaunch_add_file_check(${LAUNCH_FILE})
#     endforeach()
#   endif()
#   add_subdirectory(test)
# endif()
