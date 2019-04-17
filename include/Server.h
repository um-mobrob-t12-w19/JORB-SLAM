#ifndef SERVER_H
#define SERVER_H

#include <memory>
#include <vector>
#include <thread>
#include <atomic>
#include <string>
#include <unordered_map>
#include <map>

#include <yaml-cpp/yaml.h>

#include "GlobalLoopClosing.h"
#include "Map.h"
#include "System.h"
#include "KeyFrameDatabase.h"
#include "ORBVocabulary.h"
#include "Viewer.h"
#include "MapDrawer.h"

namespace ORB_SLAM2
{

class System;
class GlobalLoopClosing;
class Map;
class KeyFrameDatabase;
class MapDrawer;
class Viewer;

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
    void RegisterClient(System* client);

    void InsertNewKeyFrame(KeyFrame* keyframe, int offset, int sequence);
    void CopyKeyFrameMappoints(KeyFrame* keyframe);
    void CopyKeyFrameConnections(KeyFrame* keyframe);
    void FindAprilTagConnections();

public:
    YAML::Node config;

    std::vector<System*> clients;
    std::thread* globalMappingThread;
    std::atomic_bool stopped;

    ORBVocabulary* vocabulary;

    Map* globalMap;
    KeyFrameDatabase* globalDatabase;
    GlobalLoopClosing* globalLoopClosing;

    std::unordered_map<MapPoint*, MapPoint*> mapPointDictionary;
    std::unordered_map<KeyFrame*, KeyFrame*> keyFrameDictionary;

    std::map<double, KeyFrame*> timeDictionaryA;
    std::map<double, KeyFrame*> timeDictionaryB;

    Viewer* viewer;
    std::thread* viewerThread;
    MapDrawer* mapDrawer;

    KeyFrame* seqAPrevKF;
    KeyFrame* seqBPrevKF;

};

}// namespace ORB_SLAM

#endif // SYSTEM_H
