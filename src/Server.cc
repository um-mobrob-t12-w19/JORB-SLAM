#include <iostream>

#include "Server.h"

namespace ORB_SLAM2 {

Server::Server() {
    std::cout << "Starting server..." << std::endl;

    std::cout << "Server started" << std::endl;
}

void Server::Shutdown() {
    std::cout << "Shutting down server..." << std::endl;
    std::cout << "Server shut down." << std::endl;
}

void Server::RegisterClient(std::shared_ptr<System> client) {
    clients.push_back(client);

    // Inform client of server
    client->RegisterServer(std::shared_ptr<Server>(this));
}

}