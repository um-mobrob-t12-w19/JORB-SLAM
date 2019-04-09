#include <iostream>
#include <chrono>

#include "Server.h"

using namespace std::literals::chrono_literals;

namespace ORB_SLAM2 {

Server::Server() : stopped(false) {
    std::cout << "Starting server..." << std::endl;

    globalMappingThread = std::shared_ptr<std::thread>(new std::thread(&Server::Run, this));
    std::cout << "Server started" << std::endl;
}

void Server::Shutdown() {
    std::cout << "Shutting down server..." << std::endl;
    std::cout << "Server shut down." << std::endl;
}

void Server::Run() {
    while(!stopped) {
        std::cout << "Running server" << std::endl;

        std::this_thread::sleep_for(1s);
    }
}

void Server::RegisterClient(std::shared_ptr<System> client) {
    clients.push_back(client);

    // Inform client of server
    client->RegisterServer(std::shared_ptr<Server>(this));
}

}