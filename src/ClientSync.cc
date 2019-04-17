#include "ClientSync.h"

using namespace std::chrono_literals;

namespace ORB_SLAM2
{

ClientSync::ClientSync(Map* map) : map(map), finished(false)
{

}

void ClientSync::SetServer(Server* server) {
    this->server = server;
}

void ClientSync::AddKeyFrame(KeyFrame* keyFrame) {
    std::unique_lock<std::mutex> lock(syncQueueMutex);
    toSync.push(keyFrame);
}

void ClientSync::Run() 
{
    while(!finished) {
        if(server) {
            std::unique_lock<std::mutex> lock(syncQueueMutex);
            while(!toSync.empty()) {
                server->InsertNewKeyFrame(toSync.front());
                toSync.pop();
            }
        }
        std::this_thread::sleep_for(100ms);
    }
}

void ClientSync::RequestFinish() {
    finished = true;
}


}