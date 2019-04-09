#ifndef CLIENT_SYNC_H
#define CLIENT_SYNC_H

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

class ClientSync
{

public:

    ClientSync(Map* map);

    void Run();

    void RequestFinish();

private:

    Map* map;

    std::atomic_bool finished;
    
};

}// namespace ORB_SLAM

#endif // SYSTEM_H
