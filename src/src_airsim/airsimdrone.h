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

#ifndef AIRSIMDRONE_H
#define AIRSIMDRONE_H

#include "common/common_utils/StrictMode.hpp"
STRICT_MODE_OFF
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif // !RPCLIB_MSGPACK
#include "rpc/rpc_error.h"
STRICT_MODE_ON

#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include "common/common_utils/FileSystem.hpp"

#include <mutex>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <src_exploration/util_configurations.h>
#include <src_robot/gereneric_robot.h>

using namespace std;
using namespace msr::airlib;

typedef ImageCaptureBase::ImageRequest ImageRequest;
typedef ImageCaptureBase::ImageResponse ImageResponse;
typedef ImageCaptureBase::ImageType ImageType;
typedef common_utils::FileSystem FileSystem;

class AirSimDrone : public GenericRobot
{
public:
    AirSimDrone();

    void Run();

    cv::Mat GetImage();

    void InitializerMovement(float dir);

    void Land();

    void TurnAroundLeft();
    void TurnAroundRight();

    void ChangeVel(float x_vel, float y_vel, float z_vel, float yaw);
    void MoveByVelocity();

    void slam_ready(bool slam_ready);

    void StartMovement();
    void FinishMovement();
    bool idle();

    void moving(bool moving);
    bool moving();

    void RequestFinish();
    bool finish_requested();

    bool ready();

    void RequestTask(std::pair<int, std::vector<float>> task);

    float yaw();

    std::vector<float> real_position();

private:
    int turning_vel_;

    static std::mutex finish_mutex_;
    static std::mutex idle_mutex_;
    static std::mutex moving_mutex_;
    static std::mutex task_mutex_;
    static std::mutex vel_mutex_;

    thread* thread_run_;

    MultirotorRpcLibClient client_;

    pair<int, std::vector<float>> task_;

    bool moving_;
    bool idle_;
    bool finish_requested_;

    float speed_;
    float size_;
    float duration_;
    DrivetrainType driveTrain_;

    float x_vel_, y_vel_, z_vel_, yaw_vel_;

};

#endif // MAPCELL_H
