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
#include<iomanip>
#include<chrono>
#include<unistd.h>

#include<opencv2/core/core.hpp>

#include<System.h>
#include <Server.h>
using namespace std;

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeftSetA,
                vector<string> &vstrImageRightSetA, vector<string> &vstrImageLeftSetB,
				vector<string> &vstrImageRightSetB, vector<double> &vTimestamps);

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
    vector<double> vTimestamps;
    LoadImages(string(argv[3]), vstrImageLeftSetA, vstrImageRightSetA, vstrImageLeftSetB, vstrImageRightSetB, vTimestamps);

    const int nImages = vstrImageLeftSetA.size();

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    std::shared_ptr<ORB_SLAM2::System> SLAM1(new ORB_SLAM2::System(argv[1],argv[2],ORB_SLAM2::System::STEREO, string("SLAM1"), true));
	std::shared_ptr<ORB_SLAM2::System> SLAM2(new ORB_SLAM2::System(argv[1],argv[2], ORB_SLAM2::System::STEREO, string("SLAM2"), true));

    std::shared_ptr<ORB_SLAM2::Server> server(new ORB_SLAM2::Server(argv[4], argv[1]));

    server->RegisterClient(SLAM1);
    server->RegisterClient(SLAM2);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;   

    // Main loop
    cv::Mat imLeftA, imRightA, imLeftB, imRightB;
    for(int ni=0; ni<nImages; ni++)
    {
        // Read left and right images from file
        imLeftA = cv::imread(vstrImageLeftSetA[ni],CV_LOAD_IMAGE_UNCHANGED);
        imRightA = cv::imread(vstrImageRightSetA[ni],CV_LOAD_IMAGE_UNCHANGED);
		imLeftB = cv::imread(vstrImageLeftSetB[ni], CV_LOAD_IMAGE_UNCHANGED);
		imRightB = cv::imread(vstrImageRightSetB[ni], CV_LOAD_IMAGE_UNCHANGED);
        double tframe = vTimestamps[ni];

        if(imLeftA.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(vstrImageLeftSetA[ni]) << endl;
            return 1;
        }
		if (imLeftB.empty())
		{
			cerr << endl << "Failed to load image at: "
				<< string(vstrImageLeftSetB[ni]) << endl;
			return 1;
		}

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the images to the SLAM system
        SLAM1->TrackStereo(imLeftA,imRightA,tframe);
		SLAM2->TrackStereo(imLeftB,imRightB,tframe);

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];

        if(ttrack<T)
            usleep((T-ttrack)*1e6);
    }

    // Stop all threads
    SLAM1->Shutdown();
	SLAM2->Shutdown();
    server->Shutdown();

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
    SLAM1->SaveTrajectoryKITTI("CameraTrajectoryA.txt");
	SLAM2->SaveTrajectoryKITTI("CameraTrajectoryB.txt");

    return 0;
}

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeftSetA,
                vector<string> &vstrImageRightSetA, vector<string> &vstrImageLeftSetB,
				vector<string> &vstrImageRightSetB, vector<double> &vTimestamps)
{
    ifstream fTimes;
    string strPathTimeFile = strPathToSequence + "/times.txt";
    fTimes.open(strPathTimeFile.c_str());
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
            vTimestamps.push_back(t);
        }
    }

    string strPrefixLeft = strPathToSequence + "/image_0/";
    string strPrefixRight = strPathToSequence + "/image_1/";

    const int nTimes = vTimestamps.size();
	if (nTimes % 2 != 0) {
		cout << "Expecting off by one error in for loop below (stereo_kitti::LoadImages)\n";
		cout << "May also need to adjust main above (stereo_kitti::main)\n";
	}
	const int setSize = nTimes / 2;
    vstrImageLeftSetA.resize(setSize);
    vstrImageRightSetA.resize(setSize);
	vstrImageLeftSetB.resize(setSize);
	vstrImageRightSetB.resize(setSize);

    for(int i=0; i<nTimes; i++)
    {
        stringstream ss;
        ss << setfill('0') << setw(6) << i;
		if (i < setSize) {
			vstrImageLeftSetA[i] = strPrefixLeft + ss.str() + ".png";
			vstrImageRightSetA[i] = strPrefixRight + ss.str() + ".png";
		}
		else 
		{
			vstrImageLeftSetB[i-setSize] = strPrefixLeft + ss.str() + ".png";
			vstrImageRightSetB[i-setSize] = strPrefixRight + ss.str() + ".png";
		}
    }
}
