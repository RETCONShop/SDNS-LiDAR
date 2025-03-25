/////////////////////////////////////////////////////////////////////////////////////////////////////
// Name:        socket.h
// Purpose:     A broken, first draft version of the socket library for facilitating communication
//              between the client and server setup.
// Author:      Andrew Gallagher
// Created:     4/5/2024
// Revision History:
//   - 4.24.24 5:35 PM MT: Comments added
//
// Copyright 2024 Andrew Gallagher
// 
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
// 
//      http://www.apache.org/licenses/LICENSE-2.0
// 
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
/////////////////////////////////////////////////////////////////////////////////////////////////////

// A note to future capstone teams: This class isn't used. I am leaving it here so you can get 
// inspiration, and also learn from our mistakes with the first draft of this class. For the
// actual TCP socket library, please view 'TCPSocket.h'.

#include "socket.h"
#include <iostream>
#include <string>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>

Socket::Socket() : serverSocket(-1), clientSocket(-1) {}

Socket::~Socket() {
    closeConnection();
}

bool Socket::connectToServer(const std::string& serverIP, int port) {
    // Create a socket
    clientSocket = socket(AF_INET, SOCK_STREAM, 0);
    if (clientSocket == -1) {
        std::cerr << "Error creating socket\n";
        return false;
    }

    // Connect to the server
    sockaddr_in serverAddr;
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(port);
    inet_pton(AF_INET, serverIP.c_str(), &serverAddr.sin_addr);
    if (connect(clientSocket, reinterpret_cast<sockaddr*>(&serverAddr), sizeof(serverAddr)) == -1) {
        std::cerr << "Error connecting to server\n";
        closeConnection();
        return false;
    }

    std::cout << "Connected to server\n";
    return true;
}

bool Socket::sendMessage(const std::string& info) {
    // Send message to server
    ssize_t bytesSent = send(clientSocket, reinterpret_cast<const void*>(&info), sizeof(info), 0);
    if (bytesSent == -1) {
        std::cerr << "Error sending message\n";
        return false;
    }
    return true;
}

std::string Socket::receiveMessage() {
    std::string receivedInfo;
    // Receive response from server
    ssize_t bytesRead = recv(clientSocket, reinterpret_cast<void*>(&receivedInfo), sizeof(receivedInfo), 0);
    if (bytesRead == -1) {
        std::cerr << "Error receiving message\n";
        //receivedInfo.functionID = -1; // Error indicator
    }
    if (bytesRead == 0) {
        std::cerr << "Server disconnected\n";
        //receivedInfo.functionID = -1; // Disconnection indicator
    }

    return receivedInfo;
}

void Socket::closeConnection() {
    if (clientSocket != -1) {
        close(clientSocket);
        clientSocket = -1;
    }
    if (serverSocket != -1) {
        close(serverSocket);
        serverSocket = -1;
    }
}

bool Socket::startServer(int port) {
    // Create a socket
    serverSocket = socket(AF_INET, SOCK_STREAM, 0);
    if (serverSocket == -1) {
        std::cerr << "Error creating server socket\n";
        return false;
    }

    // Bind the socket
    sockaddr_in serverAddr;
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_addr.s_addr = INADDR_ANY;
    serverAddr.sin_port = htons(port);
    if (bind(serverSocket, reinterpret_cast<sockaddr*>(&serverAddr), sizeof(serverAddr)) == -1) {
        std::cerr << "Error binding server socket\n";
        closeConnection();
        return false;
    }

    // Listen for incoming connections
    if (listen(serverSocket, 10) == -1) {
        std::cerr << "Error listening for connections\n";
        closeConnection();
        return false;
    }

    std::cout << "Server is listening for connections...\n";
    return true;
}

bool Socket::acceptClient() {
    // Accept incoming connections
    sockaddr_in clientAddr;
    socklen_t clientAddrSize = sizeof(clientAddr);
    clientSocket = accept(serverSocket, reinterpret_cast<sockaddr*>(&clientAddr), &clientAddrSize);
    if (clientSocket == -1) {
        std::cerr << "Error accepting client connection\n";
        return false;
    }
    std::cout << "Client connected\n";
    return true;
}
