#include <iostream>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <opencv2/opencv.hpp>
// #include <opencv2/core/eigen.hpp>
#include <eigen3/Eigen/Dense>
#include <queue>

#include "pose_graph/transmission.cpp"
// #include "pose_graph/keyframe.h"


// Socket Programming
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <algorithm>

TransmitKeyFrame tkf;

void cb(const std_msgs::String::ConstPtr& msg){
    std::memcpy(&tkf, msg->data.c_str(), sizeof(TransmitKeyFrame));
    std::cout<<"The recvd timestamp: "<<tkf.time_stamp<<std::endl;
    std::cout<<"The recvd index: "<<tkf.index<<std::endl;
    std::cout<<"The recvd localindex: "<<tkf.local_index<<std::endl;
    std::cout<<"The recvd sequence: "<<tkf.sequence<<std::endl;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "receiver_node");
    ros::NodeHandle n("~");

    ros::Subscriber mod_kf = n.subscribe("/mod_kf_string_topic", 10, cb);
    
    ros::spin();
    return 0;
}