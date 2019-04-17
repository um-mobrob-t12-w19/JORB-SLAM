#include <iostream>
#include <chrono>

#include "Server.h"
#include "ORBmatcher.h"

using namespace std::literals::chrono_literals;

namespace ORB_SLAM2 {

Server::Server(const std::string &configFile, const string &strVocFile) : stopped(false) {
    std::cout << "Starting server..." << std::endl;

    config = YAML::LoadFile(configFile);

    std::cout <<  "Loading ORB Vocabulary. This could take a while..." << std::endl;

    vocabulary = new ORBVocabulary();

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


    // Create viewer, but don't start until global mapping
    globalMap = new Map();
    mapDrawer = new MapDrawer(globalMap, configFile);
    viewer = new Viewer(nullptr, this, nullptr, mapDrawer, nullptr, configFile, "Server");

    std::cout << "Server started" << std::endl;
}

void Server::Shutdown() {
    std::cout << "Shutting down server..." << std::endl;

    stopped = true;

    std::this_thread::sleep_for(100ms);

    std::cout << "Server shut down." << std::endl;
}

void Server::Run() {
    std::cout << "Running server" << std::endl;

    bool monocular = config["monocular"].as<bool>();

    // globalDatabase = new KeyFrameDatabase(*vocabulary);
    // globalLoopClosing = new GlobalLoopClosing(globalMap, globalDatabase, vocabulary, !monocular);

    // // Start global mapping and viewer
    // globalMappingThread = new std::thread(&GlobalLoopClosing::Run, globalLoopClosing);
    viewerThread = new std::thread(&Viewer::Run, viewer);

    std::vector<KeyFrame*> keyframesSeqA = clients[0]->mpMap->GetAllKeyFrames();
    std::vector<KeyFrame*> keyframesSeqB = clients[1]->mpMap->GetAllKeyFrames();

    std::vector<MapPoint*> mappointsSeqA = clients[0]->mpMap->GetAllMapPoints();
    std::vector<MapPoint*> mappointsSeqB = clients[1]->mpMap->GetAllMapPoints();

    // while(!stopped) {
    for(KeyFrame* keyframe : keyframesSeqA) {
        InsertNewKeyFrame(keyframe, 0);
    }

    for(KeyFrame* keyframe : keyframesSeqA) {
        CopyKeyFrameMappoints(keyframe);
    }

    for(KeyFrame* keyframe : keyframesSeqA) {
        CopyKeyFrameConnections(keyframe);
    }

    int offset = globalMap->GetAllKeyFrames().size();
    for(KeyFrame* keyframe : keyframesSeqB) {
        InsertNewKeyFrame(keyframe, offset);
    }

    for(KeyFrame* keyframe : keyframesSeqB) {
        CopyKeyFrameMappoints(keyframe);
    }

    for(KeyFrame* keyframe : keyframesSeqB) {
        CopyKeyFrameConnections(keyframe);
    }

    // globalLoopClosing->InsertKeyFrame(newKeyFrame);

    // for(KeyFrame* keyframe : keyframesSeqB) {
    //     InsertNewKeyFrame(keyframe, offset);
    //     std::this_thread::sleep_for(50ms);
    // }

}

void Server::InsertNewKeyFrame(KeyFrame* keyframe, int offset) {
    KeyFrame* newKeyFrame = new KeyFrame(keyframe, globalMap, globalDatabase);
    newKeyFrame->mnId += offset;

    globalMap->AddKeyFrame(newKeyFrame);
    keyFrameDictionary[keyframe] = newKeyFrame;
}

void Server::CopyKeyFrameMappoints(KeyFrame* keyframe) {
    KeyFrame* globalKeyFrame = keyFrameDictionary[keyframe];

    // Transfer map points
    size_t i = 0;
    for(MapPoint* point : keyframe->GetMapPointMatches()) {
        if(point == nullptr) continue;
        if(point->isBad()) continue;
        if(mapPointDictionary.find(point) == mapPointDictionary.end()) {
            // Point not in server map
            MapPoint* newMapPoint = new MapPoint(point->GetWorldPos(), globalKeyFrame, globalMap);
            newMapPoint->AddObservation(globalKeyFrame,i);
            globalKeyFrame->AddMapPoint(point,i);
            globalMap->AddMapPoint(newMapPoint);
            mapPointDictionary[point] = newMapPoint;
        } else {
            // Point in server map
            MapPoint* globalPoint = mapPointDictionary[point];
            globalPoint->AddObservation(globalKeyFrame,i);
            globalKeyFrame->AddMapPoint(globalPoint, i);
        }
    } 
}

void Server::CopyKeyFrameConnections(KeyFrame* keyframe) {

    KeyFrame* globalKeyFrame = keyFrameDictionary[keyframe];

    // Sync keyframe connections
    for(auto kfWeightPair : keyframe->mConnectedKeyFrameWeights) {
        KeyFrame* connKF = kfWeightPair.first;
        KeyFrame* globalConnection = keyFrameDictionary[connKF];
        int weight = kfWeightPair.second;

        globalKeyFrame->mConnectedKeyFrameWeights[globalConnection] = weight;
    }

    globalKeyFrame->mvpOrderedConnectedKeyFrames.reserve(keyframe->mvpOrderedConnectedKeyFrames.size());
    for(KeyFrame* orderedConnection : keyframe->mvpOrderedConnectedKeyFrames) {
        KeyFrame* globalOrderedConn = keyFrameDictionary[orderedConnection];
        globalKeyFrame->mvpOrderedConnectedKeyFrames.push_back(globalOrderedConn);
    }

    globalKeyFrame->mvOrderedWeights.reserve(keyframe->mvOrderedWeights.size());
    for(int weight : keyframe->mvOrderedWeights) {
        globalKeyFrame->mvOrderedWeights.push_back(weight);
    }

    // Sync spanning tree
    if(keyframe->mpParent) {
        KeyFrame* globalParent = keyFrameDictionary[keyframe->mpParent];
        globalKeyFrame->ChangeParent(globalParent);
    }

    for(KeyFrame* child : keyframe->mspChildrens) {
        KeyFrame* globalChild = keyFrameDictionary[child];
        globalKeyFrame->mspChildrens.insert(globalChild);
    }

    // Sync loop edges
    for(KeyFrame* loopClosure : keyframe->mspLoopEdges) {
        KeyFrame* globalLoopClosure = keyFrameDictionary[loopClosure];
        globalKeyFrame->mspLoopEdges.insert(globalLoopClosure);
    }
}

void Server::RegisterClient(System* client) {
    clients.push_back(client);

    // Inform client of server
    client->RegisterServer(this);
}


}