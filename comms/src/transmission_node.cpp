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


#define PORT 54321
#define BUFFER_SIZE 4096
#define serverIP "127.0.0.1"

std::queue<std::string> KFbuffer;
std::string KFdata;
int clientSocket = 0;


void CreateSocketandConnect(){
    // Create a socket
    // std::cout << "Trying to transmit keyframe" << std:: endl;
    clientSocket = socket(AF_INET, SOCK_STREAM, 0);
    if (clientSocket == -1) {
        std::cerr << "Socket creation failed." << std::endl;
        return;
    }

    // Set up the server address
    struct sockaddr_in serverAddress{};
    serverAddress.sin_family = AF_INET;
    serverAddress.sin_port = htons(PORT);
    serverAddress.sin_addr.s_addr = inet_addr(serverIP); // Server IP address

    // Connect to the server
    if (connect(clientSocket, (struct sockaddr *)&serverAddress, sizeof(serverAddress)) < 0) {
        std::cerr << "Connection failed." << std::endl;
        close(clientSocket);
        return;
    }

    std::cout << "Connected to the server." << std::endl;
}


void TransmitKeyFrame(const std::string& KFdata){
    // Send the total size of the serialized data to the server
    size_t dataSize = KFdata.size();
    std::cout << "Data size: " << dataSize << std::endl;
    ssize_t bytesSent = send(clientSocket, &dataSize, sizeof(dataSize), 0);
    if (bytesSent < 0) {
        std::cerr << "Failed to send data size to the server." << std::endl;
        close(clientSocket);
        return;
    }

    // Send the serialized data to the server in chunks
    ssize_t totalBytesSent = 0;
    while (totalBytesSent < dataSize) {
        ssize_t remainingBytes = dataSize - totalBytesSent;
        ssize_t bytesToSend = BUFFER_SIZE < remainingBytes ? BUFFER_SIZE : remainingBytes;
        bytesSent = send(clientSocket, KFdata.data() + totalBytesSent, bytesToSend, 0);
        if (bytesSent < 0) {
            std::cerr << "Failed to send data to the server." << std::endl;
            close(clientSocket);
            return;
        }
        totalBytesSent += bytesSent;
    }

    std::cout << "KeyFrame object sent to the server. Total Bytes sent: " << totalBytesSent << std::endl;
    
    // Close the client socket
    close(clientSocket);

    return;
}


void KFCallback(const std_msgs::String::ConstPtr& KFmsg){
    // std::cout << "inside callback" << std::endl;
    std::string KFdata = KFmsg->data;
    KFbuffer.push(KFdata);
    // ConnectandTransmitKeyFrame(KFdata);

} 


int main(int argc, char** argv) {
    ros::init(argc, argv, "ClientNode");
    ros::NodeHandle n("~");

    CreateSocketandConnect();
    ros::Subscriber KFmsg = n.subscribe("/serialized_keyframe", 10, KFCallback);


    ros::AsyncSpinner spinner(1); // Create an asynchronous spinner with one thread
    spinner.start(); // Start the spinner

    int count = 0;
    while(ros::ok()){
        count++;
        if (count<=20){
            std::cout << "count: " << count << std::endl;
            continue;
        }

        for (int i=0; i<KFbuffer.size(); i++){
            KFdata = KFbuffer.front();
            KFbuffer.pop();
            TransmitKeyFrame(KFdata);
            // std::cout << KFdata << std::endl;
        }
        // std::cout << "inside ros ok loop" << std::endl;
    }

    spinner.stop();
    ros::spin();
    return 0;
}
