#ifndef CLIENT_SYNC_H
#define CLIENT_SYNC_H

#include <memory>
#include <vector>
#include <thread>
#include <atomic>
#include <string>
#include <mutex>
#include <queue>

#include <yaml-cpp/yaml.h>

#include "GlobalLoopClosing.h"
#include "Map.h"
#include "System.h"
#include "KeyFrameDatabase.h"
#include "ORBVocabulary.h"
#include "KeyFrame.h"
#include "MapPoint.h"
#include "Server.h"

namespace ORB_SLAM2
{

class Map;
class KeyFrame;
class Server;

class ClientSync
{

public:

    ClientSync(Map* map);

    void Run();

    void SetServer(Server* server);

    void RequestFinish();

    void AddKeyFrame(KeyFrame* keyFrame);

private:

    Map* map;
    Server* server;

    std::queue<KeyFrame*> toSync;

    std::atomic_bool finished;

    std::mutex syncQueueMutex;
    
};

}// namespace ORB_SLAM

#endif // SYSTEM_H
