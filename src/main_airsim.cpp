/**
 * This file is part of LoS-Exploration.
 *
 * Copyright 2020 Diego Pittol <dpittol at inf dot ufrgs dot br> (Phi Robotics Research Lab - UFRGS)
 * For more information see <https://github.com/phir2-lab/LoS-Exploration>
 *
 * LoS-Exploration is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * LoS-Exploration is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with LoS-Exploration. If not, see <https://www.gnu.org/licenses/>.
**/

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<iomanip>

#include <chrono>
#include <ctime>

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <System.h>

#include <src_exploration/exploration.h>
#include <src_exploration/util_configurations.h>

#include <src_robot/gereneric_robot.h>

#include <src_airsim/airsimdrone.h>

#include <sys/stat.h>
#include <cstdlib>

#include <string>

#include <thread>

using namespace cv;
using namespace std;

int main(int argc, char **argv)
{
    cout << endl << "LoS-Exploration Copyright (C) 2020 - Diego Pittol, UFRGS." << endl <<
            "This program comes with ABSOLUTELY NO WARRANTY;" << endl  <<
            "This is free software, and you are welcome to redistribute it" << endl <<
            "under certain conditions. See LICENSE." << endl << endl;

    if(argc != 2)
    {
        cerr << endl << "Usage: ./main_airsim path_settings" << endl;
        return 1;
    }

    UtilConfigurations configuration;

    configuration.Load(argv[1]);

    ORB_SLAM2::System SLAM(configuration.GetString("path_to_vocabulary"), configuration.GetString("path_to_cam_calib"), ORB_SLAM2::System::MONOCULAR, configuration.GetBool("orbslam_show_full"), configuration.GetBool("orbslam_show_image"));

    GenericRobot* robot = new AirSimDrone();

    //Exploration
    Exploration exploration(configuration);
    exploration.robot(robot);
    exploration.slam(&SLAM);

    // Main loop
    robot->slam_ready(true);
    cv::Mat im;
    std::chrono::steady_clock::time_point starting_time = std::chrono::steady_clock::now();

    std::chrono::steady_clock::time_point pass = starting_time;

    int fps = configuration.GetInt("fps");
    float frequency = 0;
    if(fps > 0) frequency = 1/fps;

    while(!exploration.finished())
    {
        im = robot->GetImage();

        std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
        double tFrame= std::chrono::duration_cast<std::chrono::duration<double> >(now - starting_time).count();

        if(std::chrono::duration_cast<std::chrono::duration<double> >(now - pass).count() < frequency) continue;

        pass = now;

        // Pass the image to the SLAM system and return position to Exploration
        exploration.SetPose(SLAM.TrackMonocular(im,tFrame));
    }

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    while(robot->ready() || exploration.viewer_online())
    {
        sleep(1);
    }

    cout << endl << endl << "Exploration finished!!!"  << endl << endl;

    return 0;
}
