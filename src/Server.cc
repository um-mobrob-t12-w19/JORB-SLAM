#include <iostream>
#include <chrono>

#include "Server.h"

using namespace std::literals::chrono_literals;

namespace ORB_SLAM2 {

Server::Server(const std::string &configFile, const string &strVocFile) : stopped(false) {
    std::cout << "Starting server..." << std::endl;

    config = YAML::LoadFile(configFile);

    std::cout <<  "Loading ORB Vocabulary. This could take a while..." << std::endl;

    vocabulary = std::shared_ptr<ORBVocabulary>(new ORBVocabulary());

    const std::string txt_suffix(".txt"); 
    bool loaded;
    if (strVocFile.find(txt_suffix, strVocFile.size() - txt_suffix.size()) != string::npos)
        loaded = vocabulary->loadFromTextFile(strVocFile);
    else
        loaded = vocabulary->loadFromBinaryFile(strVocFile);
    if(!loaded)
    {
        cerr << "Wrong path to vocabulary. " << endl;
        cerr << "Falied to open at: " << strVocFile << endl;
        exit(-1);
    }
    cout << "Vocabulary loaded!" << endl << endl;

    bool monocular = config["monocular"].as<bool>();

    globalMap = std::shared_ptr<Map>(new Map());
    globalDatabase = std::shared_ptr<KeyFrameDatabase>(new KeyFrameDatabase(*vocabulary));
    globalLoopClosing = std::shared_ptr<GlobalLoopClosing>(new GlobalLoopClosing(globalMap.get(), globalDatabase.get(), vocabulary.get(), !monocular));

    // globalMappingThread = std::shared_ptr<std::thread>(new std::thread(&GlobalLoopClosing::Run, globalLoopClosing.get()));

    mapDrawer = new MapDrawer(globalMap.get(), configFile);
    viewer = new Viewer(nullptr, this, nullptr, mapDrawer, nullptr, configFile, "Server");
    viewerThread = new std::thread(&Viewer::Run, viewer);

    std::cout << "Server started" << std::endl;
}

void Server::Shutdown() {
    std::cout << "Shutting down server..." << std::endl;

    stopped = true;

    std::this_thread::sleep_for(100ms);

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

void Server::EraseMapPoint(MapPoint* mapPoint) {
    if(mapPointDictionary.find(mapPoint) != mapPointDictionary.end()) {
        mapPointDictionary[mapPoint]->SetBadFlag();
    }
}

void Server::InsertNewKeyFrame(KeyFrame* keyframe) {
    KeyFrame* newKeyFrame = new KeyFrame(keyframe, globalMap.get(), globalDatabase.get());
    
    // Transfer map points
    size_t i = 0;
    for(MapPoint* point : keyframe->GetMapPoints()) {
        if(point == nullptr) continue;
        if(point->isBad()) continue;
        
        if(mapPointDictionary.find(point) == mapPointDictionary.end()) {
            // Point not in server map
            MapPoint* newMapPoint = new MapPoint(point->GetWorldPos(), newKeyFrame, globalMap.get());
            newMapPoint->AddObservation(newKeyFrame,i);
            newKeyFrame->AddMapPoint(point,i);
            newMapPoint->ComputeDistinctiveDescriptors();
            newMapPoint->UpdateNormalAndDepth();
            globalMap->AddMapPoint(newMapPoint);
            mapPointDictionary[point] = newMapPoint;
        } else {
            // Point in server map
            newKeyFrame->AddMapPoint(mapPointDictionary[point], i);
        }
    } 

    for(KeyFrame* connectedKeyFrame : keyframe->GetConnectedKeyFrames()) {
        if(keyFrameDictionary.find(connectedKeyFrame) != keyFrameDictionary.end()) {
            int weight = keyframe->GetWeight(connectedKeyFrame);
            newKeyFrame->AddConnection(keyFrameDictionary[connectedKeyFrame], weight);
        }
    }

    globalMap->AddKeyFrame(newKeyFrame);
    newKeyFrame->ComputeBoW();
    keyFrameDictionary[keyframe] = newKeyFrame;
    newKeyFrame->UpdateConnections();

    KeyFrame* keyframeParent = keyframe->GetParent();
    if(keyframeParent) {
        if(!keyframeParent->isBad()) {
            if(keyFrameDictionary.find(keyframeParent) != keyFrameDictionary.end()) {
                newKeyFrame->ChangeParent(keyFrameDictionary[keyframeParent]);
            }
        }
    }

    globalLoopClosing->InsertKeyFrame(newKeyFrame);
}


}