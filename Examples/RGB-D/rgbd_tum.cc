/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<unistd.h>

#include<opencv2/core/core.hpp>

#include<System.h>

using namespace std;

constexpr double start_time = 105;
constexpr double end_time = 107;
constexpr double sleep_multiplier = 1; // Run at half speed

void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps);

int main(int argc, char **argv)
{
    if(argc != 9)
    {
        cerr << endl << "Usage: ./rgbd_tum path_to_vocabulary path_to_settings1 path_to_sequence1 path_to_association1 path_to_settings2 path_to_sequence2 path_to_association2 path_to_server_settings" << endl;
        return 1;
    }

    // Retrieve paths to images
    vector<string> vstrImageFilenamesRGB1;
    vector<string> vstrImageFilenamesD1;
    vector<double> vTimestamps1;
    string strAssociationFilename1 = string(argv[4]);
    LoadImages(strAssociationFilename1, vstrImageFilenamesRGB1, vstrImageFilenamesD1, vTimestamps1);
    
    vector<string> vstrImageFilenamesRGB2;
    vector<string> vstrImageFilenamesD2;
    vector<double> vTimestamps2;
    string strAssociationFilename2 = string(argv[7]);
    LoadImages(strAssociationFilename2, vstrImageFilenamesRGB2, vstrImageFilenamesD2, vTimestamps2);

    // Check consistency in the number of images and depthmaps
    int nImages = vstrImageFilenamesRGB1.size() < vstrImageFilenamesRGB2.size() ? vstrImageFilenamesRGB1.size(): vstrImageFilenamesRGB2.size(); // Select minimum
    if(vstrImageFilenamesRGB1.empty())
    {
        cerr << endl << "No images found in provided path." << endl;
        return 1;
    }
    else if(vstrImageFilenamesD1.size()!=vstrImageFilenamesRGB1.size())
    {
        cerr << endl << "Different number of images for rgb and depth." << endl;
        return 1;
    }

    std::cout << "Creating system" << std::endl;

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    std::shared_ptr<ORB_SLAM2::System> SLAM1(new ORB_SLAM2::System(argv[1],argv[2],ORB_SLAM2::System::RGBD, string("SLAM1"), true));
	std::shared_ptr<ORB_SLAM2::System> SLAM2(new ORB_SLAM2::System(argv[1],argv[5], ORB_SLAM2::System::RGBD, string("SLAM2"), true));
    
    std::shared_ptr<ORB_SLAM2::Server> server(new ORB_SLAM2::Server(argv[8], argv[1]));

    server->RegisterClient(SLAM1.get());
    server->RegisterClient(SLAM2.get());

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // cout << "Press enter to start" << std::endl;
    // cin.ignore();
    // Main loop
    cv::Mat imRGB1, imD1, imRGB2, imD2;
    size_t seqAIdx = 0;
    size_t seqBIdx = 0;
    size_t i = 0;
    while(true) // 100 frames to skip the start part (for synchronizing)
    {
        if(vTimestamps1.size() <= seqAIdx || vTimestamps2.size() <= seqBIdx) {
            cout << "Breaking" << endl;
            break;
        }
        
        double tframe1 = vTimestamps1[seqAIdx], tframe2 = vTimestamps2[seqBIdx];
        double min_time = std::min(tframe1, tframe2);

        if(min_time >= end_time) break;

        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        if(tframe1 < tframe2) {
            // Read image and depthmap from file for SLAM1
            imRGB1 = cv::imread(string(argv[3])+"/"+vstrImageFilenamesRGB1[seqAIdx],CV_LOAD_IMAGE_UNCHANGED);
            imD1 = cv::imread(string(argv[3])+"/"+vstrImageFilenamesD1[seqAIdx],CV_LOAD_IMAGE_UNCHANGED);
            
            if(imRGB1.empty())
            {
                cerr << endl << "Failed to load color image at: "
                    << string(argv[3]) << "/" << vstrImageFilenamesRGB1[seqAIdx] << endl;
                return 1;
            }
            if(imD1.empty()) {
                cerr << endl << "Failed to load depth image at: " 
                << string(argv[3]) << "/" << vstrImageFilenamesD1[seqAIdx] << endl;
                return 1;
            }

            SLAM1->TrackRGBD(imRGB1,imD1,tframe1);
            seqAIdx++;
        } else {
            //  Read image and depthmap from file for SLAM2
            imRGB2 = cv::imread(string(argv[6])+"/"+vstrImageFilenamesRGB2[seqBIdx],CV_LOAD_IMAGE_UNCHANGED);
            imD2 = cv::imread(string(argv[6])+"/"+vstrImageFilenamesD2[seqBIdx],CV_LOAD_IMAGE_UNCHANGED);

            if(imRGB2.empty())
            {
                cerr << endl << "Failed to load color image at: "
                    << string(argv[3]) << "/" << vstrImageFilenamesRGB2[seqBIdx] << endl;
                return 1;
            }
            if(imD2.empty()) {
                cerr << endl << "Failed to load depth image at: " 
                << string(argv[3]) << "/" << vstrImageFilenamesD2[seqBIdx] << endl;
                return 1;
            }

            SLAM2->TrackRGBD(imRGB2,imD2,tframe2);
            seqBIdx++;
        }

        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
        
        tframe1 = vTimestamps1[seqAIdx];
        tframe2 = vTimestamps2[seqBIdx];
        double min_time2 = std::min(tframe1, tframe2);
        double r = min_time2 - min_time;

        std::this_thread::sleep_for(sleep_multiplier * r * 1s);
    }

    // Wait for the clients to finish processing
    std::cout << "Finished running clients. Press enter to run server" << std::endl;
    std::cin.ignore();


    // Disable local mapping and loop closing threads. Keeps viewer alive
    SLAM1->ActivateLocalizationMode();
    SLAM2->ActivateLocalizationMode();

    std::cout << "Syncing maps..." << std::endl;

    server->Run();    

    std::cout << "Finished syncing maps." << std::endl;

    std::cout << "Press enter to continue" << std::endl;
    std::cin.ignore();

    // Stop all threads
    server->Shutdown();
    SLAM1->Shutdown();
    SLAM2->Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    // // Save camera trajectory
    // SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");
    // SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");   

    return 0;
}

void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps)
{
    ifstream fAssociation;
    fAssociation.open(strAssociationFilename.c_str());
    while(!fAssociation.eof())
    {
        string s;
        getline(fAssociation,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            string sRGB, sD;
            ss >> t;
            // Sync starting time
            if(t < start_time) continue;
            vTimestamps.push_back(t);
            ss >> sRGB;
            vstrImageFilenamesRGB.push_back(sRGB);
            ss >> t;
            ss >> sD;
            vstrImageFilenamesD.push_back(sD);

        }
    }
}
