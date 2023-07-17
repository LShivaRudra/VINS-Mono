#include <iostream>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <opencv2/opencv.hpp>
// #include <opencv2/core/eigen.hpp>
#include <eigen3/Eigen/Dense>
#include <queue>
#include "pose_graph/transmission.cpp"

// Socket Programming
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <algorithm>

#define PORT 54321
#define BUFFER_SIZE 4096
int clientSocket;
int serverSocket;


void CreateSocketandConnect(){
    // Create a socket
    serverSocket = socket(AF_INET, SOCK_STREAM, 0);
    if (serverSocket == -1) {
        std::cerr << "Socket creation failed." << std::endl;
        return;
    }

    // Set up the server address
    struct sockaddr_in serverAddress{};
    serverAddress.sin_family = AF_INET;
    serverAddress.sin_port = htons(PORT);
    serverAddress.sin_addr.s_addr = inet_addr("127.0.0.1"); // Server IP address

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
    clientSocket = accept(serverSocket, (struct sockaddr*)&clientAddress, &clientAddressLength);
    if (clientSocket < 0) {
        std::cerr << "Failed to accept client connection." << std::endl;
        close(serverSocket);
        return;
    }

    std::cout << "Client connected." << std::endl;
}


void ConnectandReceiveKeyFrame(){
    // Receive the total size of the serialized data
    size_t totalSize = 0;
    ssize_t bytesRead = recv(clientSocket, &totalSize, sizeof(totalSize), 0);
    if (bytesRead < 0){
        std::cerr << "Failed to receive data size from client." << std::endl;
        close(clientSocket);
        return;
    }
    // Allocate memory for the buffer
    char* buffer = new char[totalSize];
    ssize_t totalBytesRead = 0;

    // Receive the serialized data in chunks
    while (totalBytesRead < totalSize){
        ssize_t bytesReceived = recv(clientSocket, buffer + totalBytesRead, totalSize - totalBytesRead, 0);
        if (bytesReceived < 0){
            std::cerr << "Failed to receive data from client." << std::endl;
            delete[] buffer;
            close(clientSocket);
            return;
        }
        totalBytesRead += bytesReceived;
    }

    // Deserialize the received data
    TransmitKeyFrame *receivedKf = new TransmitKeyFrame();
    std::memcpy(receivedKf, buffer, sizeof(TransmitKeyFrame));
    
    // Print the received data
    std::cout<< "Size of received keyframe: " << sizeof(*receivedKf) << std::endl;
    std::cout << "Received Keyframe Index: " << receivedKf->index << std::endl;

    // Clean up
    delete[] buffer;
    delete receivedKf;
}


int main(int argc, char** argv){
    ros::init(argc, argv, "ServerNode");
    ros::NodeHandle("~");

    CreateSocketandConnect();

    ros::AsyncSpinner spinner(1); // Create an asynchronous spinner with one thread
    spinner.start(); // Start the spinner

    while(ros::ok()){
        std::cout << "inside ros ok loop" << std::endl;
        ConnectandReceiveKeyFrame();
    }

    spinner.stop();
    // ros::spin();
    close(clientSocket);
    // close(clientSocket);

    return 0;
}