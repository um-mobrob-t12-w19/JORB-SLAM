#ifndef SERVER_H
#define SERVER_H

#include <memory>
#include <vector>
#include <thread>
#include <atomic>

#include "System.h"

namespace ORB_SLAM2
{

class System;

class Server
{

public:

    // Initialize the server. It launches the global map loop closing thread
    Server();

    // Stops all threads
    void Shutdown();

    // Runs the global mapping
    void Run();

    // Registers an ORBSLAM System as a client which needs to be synced with the global map
    void RegisterClient(std::shared_ptr<System> client);

private:

    std::vector<std::shared_ptr<System>> clients;
    std::shared_ptr<std::thread> globalMappingThread;
    std::atomic_bool stopped;

};

}// namespace ORB_SLAM

#endif // SYSTEM_H
