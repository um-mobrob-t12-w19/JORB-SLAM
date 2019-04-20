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


#include <iostream>
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <chrono>
#include <unistd.h>

#include <opencv2/core/core.hpp>

#include <System.h>
#include <Server.h>
using namespace std;

constexpr size_t seq_len = 4540;
constexpr size_t seq_A_start = 0;
// constexpr size_t seq_A_end = 100;
constexpr size_t seq_A_end = 2400;
// constexpr size_t seq_B_start = 4440;
constexpr size_t seq_B_start = 2270;
constexpr size_t seq_B_end = 4540;


void LoadImages(const string &strPathToSequence, 
                vector<string> &vstrImageLeftSetA, vector<string> &vstrImageRightSetA,
                vector<string> &vstrImageLeftSetB, vector<string> &vstrImageRightSetB,
                vector<double> &vTimestampsA, vector<double> &vTimestampsB);

int main(int argc, char **argv)
{
    if(argc != 5)
    {
        cerr << endl << "Usage: ./stereo_kitti path_to_vocabulary path_to_settings path_to_sequence path_to_server_settings" << endl;
        return 1;
    }

    // Retrieve paths to images
    vector<string> vstrImageLeftSetA;
    vector<string> vstrImageRightSetA;
    vector<string> vstrImageLeftSetB;
    vector<string> vstrImageRightSetB;
    vector<double> vTimestampsA;
    vector<double> vTimestampsB;
    LoadImages(string(argv[3]), vstrImageLeftSetA, vstrImageRightSetA, vstrImageLeftSetB, vstrImageRightSetB, vTimestampsA, vTimestampsB);

    const int nImages = vstrImageLeftSetA.size();

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    std::shared_ptr<ORB_SLAM2::System> SLAM1(new ORB_SLAM2::System(argv[1],argv[2],ORB_SLAM2::System::STEREO, string("SLAM1"), true));
	std::shared_ptr<ORB_SLAM2::System> SLAM2(new ORB_SLAM2::System(argv[1],argv[2], ORB_SLAM2::System::STEREO, string("SLAM2"), true));

    std::shared_ptr<ORB_SLAM2::Server> server(new ORB_SLAM2::Server(argv[4], argv[1]));

    server->RegisterClient(SLAM1.get());
    server->RegisterClient(SLAM2.get());

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;   
    std::cin.ignore();

    // Main loop
    cv::Mat imLeftA, imRightA, imLeftB, imRightB;
    for(size_t ni = 0; ni < seq_len; ni++)
    {

        // Load images from set A
        if(ni < vstrImageLeftSetA.size()) 
        {
            imLeftA = cv::imread(vstrImageLeftSetA[ni],CV_LOAD_IMAGE_UNCHANGED);
            imRightA = cv::imread(vstrImageRightSetA[ni],CV_LOAD_IMAGE_UNCHANGED);
            if(imLeftA.empty())
            {
                cerr << endl << "Failed to load image from set A at:  " << ni << " file: " << string(vstrImageLeftSetA[ni]) << "." << endl;
                return 1;
            }
        }

        // Load images for set b
        if(ni < vstrImageLeftSetB.size()) 
        {
            imLeftB = cv::imread(vstrImageLeftSetB[ni], CV_LOAD_IMAGE_UNCHANGED);
            imRightB = cv::imread(vstrImageRightSetB[ni], CV_LOAD_IMAGE_UNCHANGED);
            if (imLeftB.empty())
            {
                cerr << endl << "Failed to load image from set B at: " << ni << " file: " << string(vstrImageLeftSetB[ni]) << "."  << endl;
                return 1;
            }
        }



        if(ni >= vstrImageLeftSetB.size() && ni > vstrImageLeftSetA.size()) {
            std::cout << "Finished both sequences, exiting." << std::endl;
            break;
        };


        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

        // Pass the images to the SLAM system
        if(ni < vstrImageLeftSetA.size()) 
        {
            SLAM1->TrackStereo(imLeftA,imRightA,vTimestampsA[ni]);
        }
        if(ni < vstrImageLeftSetB.size()) 
        {
            SLAM2->TrackStereo(imLeftB,imRightB,vTimestampsB[ni]);
        }

        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        double TA=0,TB=0;
        if(ni<nImages-1)
        {
            TA = vTimestampsA[ni+1]-vTimestampsA[ni];
            TB = vTimestampsB[ni+1]-vTimestampsB[ni];
        }
        else if(ni>0)
        {
            TA = vTimestampsA[ni]-vTimestampsA[ni-1];
            TB = vTimestampsB[ni]-vTimestampsB[ni-1];
        }

        if(ttrack<TA || ttrack<TB)
        {
            double T = TA > TB ? TA : TB;
            // usleep((T-ttrack)*1e6);
        }
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

    // Save camera trajectory
    // SLAM1->SaveTrajectoryKITTI("CameraTrajectoryA.txt");
    // SLAM2->SaveTrajectoryKITTI("CameraTrajectoryB.txt");

    return 0;
}

void LoadImages(const string &strPathToSequence, 
                vector<string> &vstrImageLeftSetA, vector<string> &vstrImageRightSetA,
                vector<string> &vstrImageLeftSetB, vector<string> &vstrImageRightSetB,
                vector<double> &vTimestampsA, vector<double> &vTimestampsB)
{

    ifstream fTimes;
    string strPathTimeFile = strPathToSequence + "/times.txt";
    int counter = 0;
    fTimes.open(strPathTimeFile.c_str());

    double seq_A_start_time;
    double seq_B_start_time;

    size_t seq_A_size = 0;
    size_t seq_B_size = 0;

    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            ss >> t;
            if (counter >= seq_A_start && counter < seq_A_end) 
            {
                if(counter == seq_A_start) seq_A_start_time = t;
                vTimestampsA.push_back(t - seq_A_start_time);
                seq_A_size++;
            }
            if (counter >= seq_B_start && counter < seq_B_end) 
            {
                if(counter == seq_B_start) seq_B_start_time = t;
                vTimestampsB.push_back(t - seq_B_start_time);
                seq_B_size++;
            }
            counter++;
        }
    }

    string strPrefixLeft = strPathToSequence + "/image_0/";
    string strPrefixRight = strPathToSequence + "/image_1/";

    vstrImageLeftSetA.resize(seq_A_size);
    vstrImageRightSetA.resize(seq_A_size);
    vstrImageLeftSetB.resize(seq_B_size);
    vstrImageRightSetB.resize(seq_B_size);

    for(size_t i = 0; i < seq_len; i++)
    {
        stringstream ss;
        ss << setfill('0') << setw(6) << i;
        if(i >= seq_A_start && i < seq_A_end)
        {
			vstrImageLeftSetA[i - seq_A_start] = strPrefixLeft + ss.str() + ".png";
			vstrImageRightSetA[i -seq_A_start] = strPrefixRight + ss.str() + ".png";
		}
        if(i >= seq_B_start && i < seq_B_end)
		{
			vstrImageLeftSetB[i - seq_B_start] = strPrefixLeft + ss.str() + ".png";
			vstrImageRightSetB[i - seq_B_start] = strPrefixRight + ss.str() + ".png";
		}
    }
}
