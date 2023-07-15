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


#define PORT 12345
#define BUFFER_SIZE 4096

std::queue<std::string> KFbuffer;
std::string KFdata;


void ConnectandReceiveKeyFrame() {
    // Create a socket
    int serverSocket = socket(AF_INET, SOCK_STREAM, 0);
    if (serverSocket == -1) {
        std::cerr << "Socket creation failed." << std::endl;
        return;
    }

    // Set up the server address
    struct sockaddr_in serverAddress{};
    serverAddress.sin_family = AF_INET;
    serverAddress.sin_port = htons(PORT);
    serverAddress.sin_addr.s_addr = INADDR_ANY;

    // Bind the socket to the server address
    if (bind(serverSocket, (struct sockaddr*)&serverAddress, sizeof(serverAddress)) < 0) {
        std::cerr << "Binding failed." << std::endl;
        close(serverSocket);
        return;
    }

    // Listen for incoming connections
    if (listen(serverSocket, 1) < 0) {
        std::cerr << "Listening failed." << std::endl;
        close(serverSocket);
        return;
    }

    std::cout << "Server is listening for incoming connections..." << std::endl;

    // Accept a client connection
    struct sockaddr_in clientAddress{};
    socklen_t clientAddressLength = sizeof(clientAddress);
    int clientSocket = accept(serverSocket, (struct sockaddr*)&clientAddress, &clientAddressLength);
    if (clientSocket < 0) {
        std::cerr << "Failed to accept client connection." << std::endl;
        close(serverSocket);
        return;
    }

    std::cout << "Client connected." << std::endl;

    // Receive the total size of the serialized data
    size_t totalSize = 0;
    ssize_t bytesRead = recv(clientSocket, &totalSize, sizeof(totalSize), 0);
    if (bytesRead < 0) {
        std::cerr << "Failed to receive data size from client." << std::endl;
        close(clientSocket);
        return;
    }

    // Allocate memory for the buffer
    char* buffer = new char[totalSize];
    ssize_t totalBytesRead = 0;

    // Receive the serialized data in chunks
    while (totalBytesRead < totalSize) {
        ssize_t bytesReceived = recv(clientSocket, buffer + totalBytesRead, totalSize - totalBytesRead, 0);
        if (bytesReceived < 0) {
            std::cerr << "Failed to receive data from client." << std::endl;
            delete[] buffer;
            close(clientSocket);
            return;
        }
        totalBytesRead += bytesReceived;
    }

    // Deserialize the received data
    TransmitKeyFrame receivedKf;
    std::memcpy(&receivedKf, buffer, sizeof(TransmitKeyFrame));
    
    // Print the received data
    std::cout << "Received Keyframe Index: " << receivedKf.index << std::endl;
    // std::cout << "Keyframe Local Index: " << receivedKf.local_index << std::endl;
    // std::cout << "Keyframe sequence: " << receivedKf.sequence << std::endl;
    // std::cout << "Keyframe timestamp: " << receivedKf.time_stamp << std::endl;
    // // std::cout << "Keyframe image: " << receivedKf.image << endl;

    // std::cout << "Keyframe brief descriptors" << std::endl;
    // for (size_t i = 0; i < receivedKf.brief_descriptors.size(); ++i) {
    //     BRIEF::bitset descriptor = receivedKf.brief_descriptors[i];
    //     std::cout << "Descriptor " << i << ": " << descriptor << std::endl;
    // }

    // Clean up
    delete[] buffer;
    close(clientSocket);
}


void ConnectandTransmitKeyFrame() {
    // Create a socket
    // std::cout << "Trying to transmit keyframe" << std:: endl;
    int clientSocket = socket(AF_INET, SOCK_STREAM, 0);
    if (clientSocket == -1) {
        std::cerr << "Socket creation failed." << std::endl;
        return;
    }

    // Set up the server address
    struct sockaddr_in serverAddress{};
    serverAddress.sin_family = AF_INET;
    serverAddress.sin_port = htons(PORT);
    serverAddress.sin_addr.s_addr = inet_addr("127.0.0.1"); // Server IP address

    // Connect to the server
    if (connect(clientSocket, (struct sockaddr *)&serverAddress, sizeof(serverAddress)) < 0) {
        std::cerr << "Connection failed." << std::endl;
        close(clientSocket);
        return;
    }

    std::cout << "Connected to the server." << std::endl;

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
    KFdata = KFmsg->data;
    // ConnectandTransmitKeyFrame(KFdata);
} 


int main(int argc, char** argv) {
    ros::init(argc, argv, "transmission_node");
    ros::NodeHandle n("~");
    ros::Subscriber KFmsg = n.subscribe("/serialized_keyframe", 10, KFCallback);
    
    std::thread ClientProcess;
    std::thread ServerProcess;

    while (ros::ok()){
        ClientProcess = std::thread(ConnectandTransmitKeyFrame);
        ServerProcess = std::thread(ConnectandReceiveKeyFrame);

        ros::spin();
    }
    return 0;
}
