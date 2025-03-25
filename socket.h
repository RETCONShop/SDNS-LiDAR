#pragma once

#include <string>

class Socket {
public:
    Socket();
    ~Socket();

    bool connectToServer(const std::string& serverIP, int port);
    bool sendMessage(const std::string& info);
    std::string receiveMessage();
    void closeConnection();

    bool startServer(int port);
    bool acceptClient();
private:
    int clientSocket;
    int serverSocket;
};

