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

#include "src_airsim/airsimdrone.h"

std::mutex AirSimDrone::finish_mutex_;
std::mutex AirSimDrone::idle_mutex_;
std::mutex AirSimDrone::moving_mutex_;
std::mutex AirSimDrone::task_mutex_;
std::mutex AirSimDrone::vel_mutex_;

AirSimDrone::AirSimDrone(){
    finish_requested_ = false;
    idle_ = true;

    task_ = make_pair(AirSimDrone::WAIT, std::vector<float>{0});

    turning_vel_ = 3;

    x_vel_ = y_vel_ = z_vel_ = yaw_vel_ = 0;

    speed_ = 1.0f;
    size_ = 10.0f;
    duration_ = size_ / speed_;
    driveTrain_ = DrivetrainType::MaxDegreeOfFreedom;

    try {
        client_.confirmConnection();
        client_.enableApiControl(true);
        client_.armDisarm(true);
        thread_run_ = new thread(&AirSimDrone::Run, this);
    }
    catch (rpc::rpc_error&  e) {
        std::string msg = e.get_error().as<std::string>();
        std::cout << "ROBOT: " << "Exception raised by the API, something went wrong." << std::endl << "ROBOT: " << msg << std::endl;
    }
}

cv::Mat AirSimDrone::GetImage()
{
    return cv::imdecode(client_.simGetImage("0", ImageType::Scene), CV_LOAD_IMAGE_COLOR);
}

void AirSimDrone::Run()
{
    while(!slam_ready_){ //Waiting SLAM starts
        std::this_thread::sleep_for(std::chrono::duration<double>(5));
    };

    unique_lock<mutex> task_lock(task_mutex_);

    float takeoffTimeout = 4;
    client_.takeoffAsync(takeoffTimeout)->waitOnLastTask();

    std::this_thread::sleep_for(std::chrono::duration<double>(5));
    client_.hoverAsync()->waitOnLastTask();

    ready_ = true;

    task_lock.unlock();

    while(!finish_requested()){
        task_lock.lock();

        switch (task_.first)
        {
            case AirSimDrone::INITIALIZE:
                StartMovement();
                InitializerMovement(task_.second[0]);
            break;

            case AirSimDrone::TURN:
                StartMovement();
                if(task_.second[0] == 1)
                    TurnAroundRight();
                else
                    TurnAroundLeft();
            break;

            case AirSimDrone::MOVEBYVEL:
                StartMovement();
                MoveByVelocity();
            break;
        }

        task_lock.unlock();
        std::this_thread::sleep_for(std::chrono::duration<double>(1));
    };

    Land();

}

void AirSimDrone::ChangeVel(float x_vel, float y_vel, float z_vel, float yaw)
{
    unique_lock<mutex> vel_lock(vel_mutex_);

    try {
        x_vel_ = x_vel;
        y_vel_ = y_vel;
        z_vel_ = z_vel;
        yaw_vel_ = yaw;
    }
    catch (...) {
        std::cout << "ERROR: ChangeVel" << std::endl;
    }
}

void AirSimDrone::MoveByVelocity()
{
	// NOTICE
	// Here we have a simple control idea that moves the robot using
	// velocities in the three axes and a yaw velocity too.
	// This can be changed to more sophisticated control approaches.
	
    std::cout << "ROBOT: " << "Moving by Velocities" << std::endl;

    unique_lock<mutex> vel_lock(vel_mutex_);
    vel_lock.unlock();

    moving(true);

    while(moving())
    {
        if(finish_requested()) return;

        vel_lock.lock();
        try {
            YawMode yaw_mode(true, yaw_vel_);
            client_.moveByVelocityAsync(x_vel_, y_vel_, z_vel_, duration_, driveTrain_, yaw_mode);
        }
        catch (...) {
        }
        vel_lock.unlock();

        std::this_thread::sleep_for(std::chrono::duration<double>(0.2));
    }

    FinishMovement();
}

void AirSimDrone::TurnAroundRight()
{
	// NOTICE
	// Here we have a simple control idea that moves the robot inside
	// a virtual box while rotates.
	// This can be changed to more sophisticated control approaches.
	
    std::cout << "ROBOT: " << "Turning Right" << std::endl;

    auto position = client_.getMultirotorState().getPosition();

    YawMode yaw_mode(true,turning_vel_);

    float range = 0.25;
    float x_max = position.x()+range;
    float x_min = position.x()-range;
    float y_max = position.y()+range;
    float y_min = position.y()-range;
    float z_max = position.z()+range;
    float z_min = position.z()-range;

    float x_delta = 0.5;
    float y_delta = 0.4;
    float z_delta = 0.3;

    float x_vel = x_delta;
    float y_vel = y_delta;
    float z_vel = z_delta;

    moving(true);

    while(moving())
    {
        if(finish_requested()) return;

        if(x_min > client_.getMultirotorState().getPosition().x()) x_vel = x_delta;
        else if(x_max < client_.getMultirotorState().getPosition().x()) x_vel = -x_delta;

        if(y_min > client_.getMultirotorState().getPosition().y()) y_vel = y_delta;
        else if(y_max < client_.getMultirotorState().getPosition().y()) y_vel = -y_delta;

        if(z_min > client_.getMultirotorState().getPosition().z()) z_vel = z_delta;
        else if(z_max < client_.getMultirotorState().getPosition().z()) z_vel = -z_delta;

        client_.moveByVelocityAsync(x_vel, y_vel, z_vel, duration_, driveTrain_, yaw_mode);

        std::this_thread::sleep_for(std::chrono::duration<double>(0.1));
    }

    client_.moveByVelocityAsync(0, 0, 0, duration_, driveTrain_, YawMode(true, 0));

    FinishMovement();
}

void AirSimDrone::TurnAroundLeft()
{
	// NOTICE
	// Here we have a simple control idea that moves the robot inside
	// a virtual box while rotates.
	// This can be changed to more sophisticated control approaches.
	
    std::cout << "ROBOT: " << "Turning Left" << std::endl;

    auto position = client_.getMultirotorState().getPosition();

    YawMode yaw_mode(true,-turning_vel_);

    float range = 0.25;
    float x_max = position.x()+range;
    float x_min = position.x()-range;
    float y_max = position.y()+range;
    float y_min = position.y()-range;
    float z_max = position.z()+range;
    float z_min = position.z()-range;

    float x_delta = 0.4;
    float y_delta = 0.3;
    float z_delta = 0.2;

    float x_vel = x_delta;
    float y_vel = y_delta;
    float z_vel = z_delta;

    moving(true);

    while(moving())
    {
        if(finish_requested()) return;

        if(x_min > client_.getMultirotorState().getPosition().x()) x_vel = x_delta;
        else if(x_max < client_.getMultirotorState().getPosition().x()) x_vel = -x_delta;

        if(y_min > client_.getMultirotorState().getPosition().y()) y_vel = y_delta;
        else if(y_max < client_.getMultirotorState().getPosition().y()) y_vel = -y_delta;

        if(z_min > client_.getMultirotorState().getPosition().z()) z_vel = z_delta;
        else if(z_max < client_.getMultirotorState().getPosition().z()) z_vel = -z_delta;

        client_.moveByVelocityAsync(x_vel, y_vel, z_vel, duration_, driveTrain_, yaw_mode);

        std::this_thread::sleep_for(std::chrono::duration<double>(0.1));
    }

    client_.moveByVelocityAsync(0, 0, 0, duration_, driveTrain_, YawMode(true, 0));    

    FinishMovement();
}

float AirSimDrone::yaw()
{
    auto orientation = client_.getMultirotorState().getOrientation();
    float z = orientation.z();
    float y = orientation.y();
    float x = orientation.x();
    float w = orientation.w();
    float ysqr = y * y;

    float t3 = 2 * (w*z + x*y);
    float t4 = 1 - 2 * (ysqr + z*z);
    return atan2(t3, t4);
}

void AirSimDrone::InitializerMovement(float dir)
{
	// NOTICE
	// Here we have a simple control idea that moves the robot to do
	// a predefined initializer movement.
	// This can be changed to more sophisticated control approaches.
	
    auto position = client_.getMultirotorState().getPosition();
    float z = position.z();

    float yaw = this->yaw();
    float x_delta = cos(yaw)*1.2;
    float y_delta = sin(yaw)*1.2;

    if(dir == 99){
        std::cout << "ROBOT: " << "Initializer movement (full)" << std::endl;
        if(finish_requested()) return;
        client_.moveToPositionAsync(-x_delta, -y_delta, z, 1)->waitOnLastTask();
        std::this_thread::sleep_for(std::chrono::duration<double>(1));
        if(finish_requested()) return;
        client_.moveToPositionAsync(x_delta, y_delta, z, 1)->waitOnLastTask();
        std::this_thread::sleep_for(std::chrono::duration<double>(1));
        if(finish_requested()) return;
        client_.moveToPositionAsync(0, 0, z, 1)->waitOnLastTask();
        std::this_thread::sleep_for(std::chrono::duration<double>(1));
    }
    else if (dir == 1){
        std::cout << "ROBOT: " << "Initializer movement (backward)" << std::endl;
        if(finish_requested()) return;
        client_.moveToPositionAsync(-x_delta, -y_delta, z, 1)->waitOnLastTask();
        std::this_thread::sleep_for(std::chrono::duration<double>(1));
    }
    else if (dir == -1){
        std::cout << "ROBOT: " << "Initializer movement (forward)"  << std::endl;
        if(finish_requested()) return;
        client_.moveToPositionAsync(x_delta, y_delta, z, 1)->waitOnLastTask();
        std::this_thread::sleep_for(std::chrono::duration<double>(1));
    }
    else if (dir == 0){
        std::cout << "ROBOT: " << "Initializer movement (center)" << std::endl;
        if(finish_requested()) return;
        client_.moveToPositionAsync(0, 0, z, 1)->waitOnLastTask();
        std::this_thread::sleep_for(std::chrono::duration<double>(1));
    }

    client_.hoverAsync()->waitOnLastTask();

    FinishMovement();
}

void AirSimDrone::Land()
{
    std::cout << "ROBOT: " << "Landing UAV... " << std::endl;

    client_.landAsync();

    client_.waitOnLastTask();

    std::cout << "ROBOT: Done!!!" << std::endl;

    client_.armDisarm(false);

    ready_ = false;
}

void AirSimDrone::StartMovement()
{
    unique_lock<mutex> lock(idle_mutex_);
    idle_ = false;
}

void AirSimDrone::FinishMovement()
{
    auto position = client_.getMultirotorState().getPosition();
    float old_x, old_y, old_z;

    do
    {
        client_.moveByVelocityAsync(0, 0, 0, duration_, driveTrain_, YawMode(true, 0));
        old_x = position.x();
        old_y = position.y();
        old_z = position.z();
        std::this_thread::sleep_for(std::chrono::duration<double>(0.3));
        position = client_.getMultirotorState().getPosition();
    }
    while(sqrt(pow(old_x - position.x(),2) + pow(old_y-position.y(),2) + pow(old_z-position.z(),2)) > 0.005);

    task_ = make_pair(AirSimDrone::WAIT, std::vector<float>{0});
    unique_lock<mutex> lock(idle_mutex_);
    idle_ = true;

    std::cout << "ROBOT: " << "Movement finished!" << std::endl;
}

bool AirSimDrone::idle()
{
    unique_lock<mutex> lock(idle_mutex_);
    return idle_;
}

void AirSimDrone::moving(bool moving)
{
    unique_lock<mutex> lock(moving_mutex_);
    moving_ = moving;
}

bool AirSimDrone::moving()
{
    unique_lock<mutex> lock(moving_mutex_);
    return moving_;
}

void AirSimDrone::RequestFinish()
{
    unique_lock<mutex> lock(finish_mutex_);
    finish_requested_ = true;
}

bool AirSimDrone::finish_requested()
{
    unique_lock<mutex> lock(finish_mutex_);
    return finish_requested_;
}

bool AirSimDrone::ready()
{
    return ready_;
}

void AirSimDrone::RequestTask(std::pair<int, std::vector<float>> task)
{
    while(!this->idle()) std::this_thread::sleep_for(std::chrono::duration<double>(1));
    unique_lock<mutex> lock(task_mutex_);
    task_ = task;
}

std::vector<float> AirSimDrone::real_position()
{
    auto position = client_.getMultirotorState().getPosition();
    return std::vector<float>{position.x(), position.y(), position.z()};
}
