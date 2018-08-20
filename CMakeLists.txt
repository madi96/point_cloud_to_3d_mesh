cmake_minimum_required(VERSION 2.8.3)
project(point_cloud_to_3d_mesh)



set(CATKIN_COMMON_COMPONENTS cv_bridge
                             roscpp
                             rospy
                             std_msgs
                             message_generation
                             image_transport
                             image_geometry
                             tf
			     pcl_ros
                             geometry_msgs
                             sensor_msgs
			     rgbd_utils
                             ) 

find_package(catkin REQUIRED COMPONENTS ${CATKIN_COMMON_COMPONENTS})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3 -fPIC")
## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS filesystem)
find_package(OpenCV 2.4.8 REQUIRED)
find_package(PCL 1.8 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
#find_library(RGBD_UTILS_LIBRARY NAMES rgbd_utils)

## Generate messages in the 'msg' folder
# add_message_files(
#          FILES
#   )
## Generate actions in the 'action' folder
# add_action_files(
#         FILES
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#         DEPENDENCIES
#         std_msgs
#         sensor_msgs
#         geometry_msgs
# )
catkin_package(
        INCLUDE_DIRS include
        CATKIN_DEPENDS cv_bridge roscpp rospy std_msgs message_runtime rgbd_utils  sensor_msgs 
        DEPENDS system_lib
)

include_directories(include
        ${catkin_INCLUDE_DIRS}
        ${OpencCV_INCLUDE_DIRS}
)


## Declare a C++ executable
add_executable(PointCloudTo3dMesh src/PointCloudTo3dMesh.cpp)
add_dependencies(PointCloudTo3dMesh ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(PointCloudTo3dMesh
  ${catkin_LIBRARIES} ${PCL_LIBRARIES}
)