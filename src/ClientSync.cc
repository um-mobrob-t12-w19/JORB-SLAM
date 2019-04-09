#include "ClientSync.h"

using namespace std::chrono_literals;

namespace ORB_SLAM2
{

ClientSync::ClientSync(Map* map) : map(map), finished(false)
{

}

void ClientSync::Run() 
{
    while(!finished) {
        std::this_thread::sleep_for(5s);
    }
}

void ClientSync::RequestFinish() {
    finished = true;
}


}