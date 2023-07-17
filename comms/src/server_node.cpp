#include <iostream>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>

#define PORT 54321
#define BUFFER_SIZE 4096

int serverSocket;
int clientSocket;

void CreateServerSocket() {
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
}

void AcceptClientConnection() {
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

void ReceiveStringFromClient() {
    // Receive the string from the client
    char buffer[BUFFER_SIZE];
    ssize_t bytesRead = recv(clientSocket, buffer, BUFFER_SIZE, 0);
    if (bytesRead < 0) {
        std::cerr << "Failed to receive data from client." << std::endl;
        close(clientSocket);
        close(serverSocket);
        return;
    }

    // Print the received string
    std::string receivedString(buffer, bytesRead);
    // std::cout << "Received string from client: " << receivedString << std::endl;

    // Send a reply to the client
    std::string reply = "Message received by the server";
    ssize_t bytesSent = send(clientSocket, reply.c_str(), reply.length(), 0);
    if (bytesSent < 0) {
        std::cerr << "Failed to send reply to the client." << std::endl;
    } else {
        std::cout << "Reply sent to the client. Total bytes sent: " << bytesSent << std::endl;
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "ServerNode");
    ros::NodeHandle("~");

    CreateServerSocket();

    ros::AsyncSpinner spinner(1); // Create an asynchronous spinner with one thread
    spinner.start(); // Start the spinner

    // Accept client connection
    AcceptClientConnection();

    while (ros::ok()) {
        ReceiveStringFromClient();
    }

    close(clientSocket);
    close(serverSocket);
    spinner.stop();
    ros::spin();
    return 0;
}