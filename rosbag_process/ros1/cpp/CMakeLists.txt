cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

IF(NOT ROS_BUILD_TYPE)
  SET(ROS_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${ROS_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native")


set(CMAKE_CXX_STANDARD 14)
add_definitions(-DCOMPILEDWITHC14)

find_package(OpenCV 4 QUIET)
if(NOT OpenCV_FOUND)
   message(FATAL_ERROR "OpenCV > 4 not found.")
endif()


find_package(Eigen3 3.1.0 REQUIRED)
find_package(Boost COMPONENTS system filesystem REQUIRED)


include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/../../
${PROJECT_SOURCE_DIR}/../../include
)

set(LIBS 
${OpenCV_LIBS} 
${EIGEN3_LIBS}
${Boost_LIBRARIES}
)


# Node for stereo camera
rosbuild_add_executable(stereo_imu
src/ros_stereo_imu.cc
)

target_link_libraries(stereo_imu
${LIBS}
)


# Node for stereo camera
rosbuild_add_executable(stereo
src/ros_stereo.cc
)

target_link_libraries(stereo
${LIBS}
)
# # Node for RGB-D camera
# rosbuild_add_executable(RGBD
# src/ros_rgbd.cc
# )

# target_link_libraries(RGBD
# ${LIBS}
# )


# # Node for RGB-D grid map camera
# rosbuild_add_executable(RGBD_GRIDMAP
# src/ros_rgbd_gridmap.cc
# )

# target_link_libraries(RGBD_GRIDMAP
# ${LIBS}
# )

