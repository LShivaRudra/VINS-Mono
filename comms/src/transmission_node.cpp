#include <iostream>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <opencv2/opencv.hpp>
// #include <opencv2/core/eigen.hpp>
#include <eigen3/Eigen/Dense>
#include <queue>

// #include "pose_graph/transmission.cpp"


// Socket Programming
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <algorithm>


#define PORT 12345
#define BUFFER_SIZE 4096

std::queue<std::string> KFbuffer;

void ConnectandTransmitKeyFrame(const std::string& SerializedData) {
    // Create a socket
    int clientSocket = socket(AF_INET, SOCK_STREAM, 0);
    if (clientSocket == -1) {
        std::cerr << "Socket creation failed." << std::endl;
        return;
    }

    // Set up the server address
    struct sockaddr_in serverAddress{};
    serverAddress.sin_family = AF_INET;
    serverAddress.sin_port = htons(PORT);
    serverAddress.sin_addr.s_addr = inet_addr("10.2.129.86"); // Server IP address

    // Connect to the server
    if (connect(clientSocket, (struct sockaddr *)&serverAddress, sizeof(serverAddress)) < 0) {
        std::cerr << "Connection failed." << std::endl;
        close(clientSocket);
        return;
    }

    std::cout << "Connected to the server." << std::endl;

    // Send the total size of the serialized data to the server
    size_t dataSize = SerializedData.size();
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
        bytesSent = send(clientSocket, SerializedData.data() + totalBytesSent, bytesToSend, 0);
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


void keyframecallback(const std_msgs::String& serializedKF){
    KFbuffer.push(serializedKF.data);
} 


int main(int argc, char** argv) {
    ros::init(argc, argv, "pose_graph");
    ros::NodeHandle n("~");
    ros::Subscriber serializedKF = n.subscribe("/serialized_keyframe", 10, keyframecallback);
    for (int i =0; i < sizeof(KFbuffer); i++){
        // ConnectandTransmitKeyFrame(i);
        std::cout << "inside for loop to transmit serializedKF" << std::endl;
    }
    return 0;
}
