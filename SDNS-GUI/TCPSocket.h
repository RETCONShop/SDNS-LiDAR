/////////////////////////////////////////////////////////////////////////////////////////////////////
// Name:        TCPSocket.h
// Purpose:     A simple TCP Socket manager designed to run in threads, designed for the SDNS-GUI
//              and TurtleBot robot
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

#include <iostream>
#include <thread>
#include <string>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <mutex>
#include "GUISupport.h"

class TCPSocket {
public:
    TCPSocket(int port, std::string ip_address){
	this->port = port;
	this->ip_address = ip_address; 
    }

    // Starts the server on the machine. This calls the server loop function, then
    // detaches it from the main process to run independently from the main code
    void startServer() {
        std::thread serverThread(&TCPSocket::serverLoop, this);
        serverThread.detach();
    }

    // This does the same as the startServer() function, except is designed to run 
    // on the client instead of the server due to this class being shared between both
    void startClient() {
        std::thread clientThread(&TCPSocket::clientLoop, this);
        clientThread.detach();
    }

    // This function queues up a message to be sent over the socket. Works bi-directionally.
    void sendMessage(const std::string& message) {
        //std::lock_guard<std::mutex> lock(clientMutex);
        if (clientSocket != -1) {
            send(clientSocket, message.c_str(), message.length(), 0);
        }
    }
    
    // This function will try and get a response that was received by the server
    std::string getResponse() {
        std::lock_guard<std::mutex> lock(serverMutex);
        std::string responseClone = serverResponse;
        serverResponse = "";
        return responseClone;
    }

    // Client only function. This will try and send a message from the client to the connected server
    // over the TCP Socket
    void clientSendStuff(std::string contents) {
        bool tryAgain = true;
            while(tryAgain) {
                try {
                if (!clientMutex.try_lock()) {
                    tryAgain = true;
                    throw 100;
                } else {
                    tryAgain = false;
                }
            } catch (int err) {
                std::cout << "Error posting new content, mutex is locked" << std::endl;
            }
        }
        
        dataToSend = contents;
        hasSomethingToSend = true;
        clientMutex.unlock();
    }


private:
    // Class declaration of LightSerial
    LightSerial lsr;

    // Main instance of the server loop. This function will run in the detached thread,
    // and initialize the TCP socket based on the initialized parameters of the class
    void serverLoop() {
        int serverSocket = socket(AF_INET, SOCK_STREAM, 0);
        if (serverSocket == -1) {
            std::cerr << "Failed to create server socket" << std::endl;
            //qDebug("Failed to create server socket");
            return;
        } else {
            std::cout << "Successfully created server socket" << std::endl;
            //qDebug("Successfully created server socket");
        }

        struct sockaddr_in serverAddress;
        serverAddress.sin_family = AF_INET;
        serverAddress.sin_addr.s_addr = INADDR_ANY;
        serverAddress.sin_port = htons(port);

        if (bind(serverSocket, (struct sockaddr*)&serverAddress, sizeof(serverAddress)) == -1) {
            std::cerr << "Failed to bind server socket" << std::endl;
            //qDebug("Failed to bind server socket");
            close(serverSocket);
            return;
        } else {
            std::cout << "Successfully bound server socket" << std::endl;
            //qDebug("Successfully bound server socket");
        }

        if (listen(serverSocket, 1) == -1) {
            std::cerr << "Failed to listen on server socket" << std::endl;
            //qDebug("Failed to listen on server socket");
            close(serverSocket);
            return;
        } else {
            std::cout << "Listening on server socket" << std::endl;
            //qDebug("Listening on server socket");
        }

        int clientSocket = accept(serverSocket, nullptr, nullptr);
        int loopCounter = 0;
        std::string recvCompiled = "";

        while (!closeServer) {
            // Check the status of the client socket
            if (clientSocket == -1) {
                std::cerr << "Failed to accept client connection" << std::endl;
                continue;
            } else {
                std::cout << "Successfully accepted client connection" << std::endl;
            }

            // Try to listen for data from the client
            try {
                char buffer[4096];
                ssize_t bytes_received = 0;
                recvCompiled.clear();

                // Spin until exit flag is received
                while (true) {
                    bytes_received = read(clientSocket, buffer, sizeof(buffer));
                    if (bytes_received <= 0) {
                        break;
                    }
                    recvCompiled.append(buffer, bytes_received);

                    // Check if the exit flag has been received, then remove it from the received data string
                    if (recvCompiled.find("END_OF_DATA") != std::string::npos) {
                        recvCompiled.erase(recvCompiled.find("END_OF_DATA"), 11);
                        break;
                    }
                }

                // Error condition if no data is received from the client (client crash)
                if (bytes_received < 0) {
                    close(serverSocket);
                    closeServer = true;
                    throw 1;
                }

                // Send something back to the client to keep the conversation alive
                std::string sendSomething = "OK";
                if (send(clientSocket, sendSomething.c_str(), sendSomething.length(), 0) == -1) {
                    throw 2;
                }

                // Save the data to a buffer file
                saveMessage(recvCompiled);

            } catch (int err) {
                switch (err) {
                    case 1: std::cerr << "Error receiving data, suspected client close/crash" << std::endl;
                    case 2: std::cerr << "Failed to send response to client" << std::endl;

                }
            }
        }

        // Safe shutdown of the server
        close(serverSocket);
        closeServer = false;
        std::cout << "Server closed." << std::endl;
        watchdog.bufferTally = 0;
    }

    // Main client loop. This will attempt a connection to the server based on the 
    // parameters the class was initialized with, and immediately start sending data.
    // This version is different on the actual robot, due to changes needing to be made
    // for ROS2, but this is still a good starting point.
    void clientLoop() {
        // Initialize the client socket
        int clientSocket = socket(AF_INET, SOCK_STREAM, 0);
        if (clientSocket == -1) {
            std::cerr << "Failed to create client socket" << std::endl;
            return;
        }

        // Convert ip_address into a const char for usage in inet_addr
        const char * ip_address_c = ip_address.c_str();

        // Configure the client socket
        struct sockaddr_in serverAddress;
        serverAddress.sin_family = AF_INET;
        serverAddress.sin_addr.s_addr = inet_addr(ip_address_c);
        serverAddress.sin_port = htons(port);

        // Sleep time (1 second)
        unsigned int microsecond = 1000000;

        // Spin trying to connect to the server (retrty every half second)
        while (connect(clientSocket, (struct sockaddr*)&serverAddress, sizeof(serverAddress)) == -1) {
            std::cerr << "Failed to connect to server" << std::endl;
            close(clientSocket);
            usleep(0.5 * microsecond);
        }

        // Make sure the client mutex is unlocked before entering the main client loop
        clientMutex.unlock();

        while (!closeClient) {
            // Checks if the client has something to send to the server, if not, spin in idle
            if (hasSomethingToSend) {
                try {
                    // Try and lock the client mutex. True means it was able to lock, false means it was unable to lock
                    if (!clientMutex.try_lock()) {
                        throw 1;
                    }

                    // Variables for bytes sent and total sent
                    ssize_t bytes_sent = 0;
                    ssize_t total_sent = 0;

                    // Create a char array from the dataToSend string
                    const char* data_ptr = dataToSend.c_str();

                    // Get the total length of the dataToSend, and store it in a size_t length variable
                    size_t remaining = dataToSend.length();

                    // Loop until the total data sent is the length of the desired amount of data being sent
                    while (total_sent < dataToSend.length()) {
                        bytes_sent = write(clientSocket, data_ptr, remaining);
                        
                        // Throw if no data is sent (generic error)
                        if (bytes_sent < 0) {
                            close(clientSocket);
                            throw 2;
                        }

                        total_sent += bytes_sent;
                        data_ptr += bytes_sent;
                        remaining -= bytes_sent;
                    }

                    // Send the exit flag to the server, indicating data transmission is complete, and removing the flag from the received message
                    std::string termination_msg = "END_OF_DATA";
                    bytes_sent = write(clientSocket, termination_msg.c_str(), termination_msg.length());
                    /*
                    if (bytes_sent < 0) {
                        close(clientSocket);
                        throw 3;
                    }
                    */
                    
                    // Free up the sending flag, allowing for a new message to be sent
                    hasSomethingToSend = false;

                    // Unlock the client mutex
                    clientMutex.unlock();
                }
                catch (int err) {
                    switch (err) {
                        case 1: std::cout << "Mutex is locked in client loop" << std::endl;
                        case 2: std::cout << "Error sending data" << std::endl;
                        case 3: std::cerr << "Error sending termination message" << std::endl;
                    }
                }
            }
        }
    }

    // Misc variable declarations for use inside the class
    int port;
    std::string ip_address;
    std::string serverResponse;
    std::mutex serverMutex;
    int clientSocket = -1;
    std::mutex clientMutex;
    bool beVerbose = true;
    std::string dataToSend;
    bool hasSomethingToSend = false;
    bool closeClient = false;
    bool closeServer = false;
};
