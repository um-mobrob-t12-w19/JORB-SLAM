#ifndef SERVER_H
#define SERVER_H

#include <memory>
#include <vector>
#include <thread>
#include <atomic>
#include <string>

#include <yaml-cpp/yaml.h>

#include "GlobalLoopClosing.h"
#include "Map.h"
#include "System.h"
#include "KeyFrameDatabase.h"
#include "ORBVocabulary.h"

namespace ORB_SLAM2
{

class System;
class GlobalLoopClosing;
class Map;
class KeyFrameDatabase;

class Server
{

public:

    // Initialize the server. It launches the global map loop closing thread
    Server(const std::string &configFile, const string &strVocFile);

    // Stops all threads
    void Shutdown();

    // Runs the global mapping
    void Run();

    // Registers an ORBSLAM System as a client which needs to be synced with the global map
    void RegisterClient(std::shared_ptr<System> client);

private:
    YAML::Node config;

    std::vector<std::shared_ptr<System>> clients;
    std::shared_ptr<std::thread> globalMappingThread;
    std::atomic_bool stopped;

    std::shared_ptr<ORBVocabulary> vocabulary;

    std::shared_ptr<Map> globalMap;
    std::shared_ptr<KeyFrameDatabase> globalDatabase;
    std::shared_ptr<GlobalLoopClosing> globalLoopClosing;

};

}// namespace ORB_SLAM

#endif // SYSTEM_H
