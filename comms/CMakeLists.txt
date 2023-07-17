cmake_minimum_required(VERSION 3.0.2)
project(comms)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
#-DEIGEN_USE_MKL_ALL")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    nav_msgs
    cv_bridge
    roslib
    pose_graph
    camera_model
    )

find_package(OpenCV)
# find_package(pose_graph REQUIRED)

find_package(Ceres REQUIRED)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3)

include_directories(${catkin_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS}  ${EIGEN3_INCLUDE_DIR})
# include_directories("/home/anagh/vinsws/src/VINS-Mono/pose_graph/include")

catkin_package()

add_executable(ClientNode
    src/transmission_node.cpp
    )

add_executable(ServerNode
    src/server_node.cpp    
)

target_link_libraries(ClientNode ${catkin_LIBRARIES}  ${OpenCV_LIBS}) 
target_link_libraries(ServerNode ${catkin_LIBRARIES}  ${OpenCV_LIBS}) 