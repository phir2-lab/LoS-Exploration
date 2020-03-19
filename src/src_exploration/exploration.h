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

#ifndef EXPLORATION_H
#define EXPLORATION_H

#include <iostream>
#include <vector>
#include <thread>
#include <math.h>
#include <unistd.h>

#include "src_orbslam/System.h"

#include "src_airsim/airsimdrone.h"

#include "src_exploration/viewer.h"
#include "src_exploration/local_map.h"
#include "src_exploration/path_planner.h"
#include "src_exploration/logger.h"
#include "src_exploration/util_configurations.h"

class Exploration
{
public:
    Exploration(UtilConfigurations configuration);

    void Run();

    void Reset();

    void SetPose(cv::Mat p);

    void slam(ORB_SLAM2::System *slam_);

    void CreateLocalMap();
    std::vector<ORB_SLAM2::KeyFrame*> OpenKeyframes();

    void UpdateLocalMap();

    void Planning();

    bool UpdateUavGlobalPoseInLocal();

    void UpdateLocalMapDuringNavigation();

    void Navigate();

    void StopRobot();

    void InitializerMovement(bool get_scale);

    void TurningAround();

    bool CheckFinished();

    bool finished();

    void StoreDecision(int d);
    int get_last_decision();

    void robot(GenericRobot *value);

    cv::Mat uav_global_position();
    cv::Mat uav_global_pose_mat();

    bool viewer_online();

    enum ExplorationStates{
        NOT_READY=-1,
        WAITING=0,
        UPDATING_LOCAL_MAP=1,
        PLANNING=2,
        NAVIGATING=3,
        LOOKING_AROUND=4,
        CHECKING_FINISH=5,
        START_MOVEMENT=6,
        FINISHED=99
    };

    static std::mutex pose_mutex;

private:

    GenericRobot* robot_;

    ExplorationViewer* viewer_;

    ORB_SLAM2::System* slam_;

    LocalMap* local_map_;

    PathPlanner* path_planner_;

    Logger* logger_;

    cv::Mat uav_global_position_;
    cv::Mat uav_global_pose_mat_;

    cv::Mat slam_start_position_;
    std::vector<float> robot_start_position_;
    std::vector<std::pair<float, float>> scale_historic_;

    bool finished_;

    bool initialized_;

    bool global_navigation_;

    pair<float, bool> initial_yaw_;

    bool global_pose_changed_;

    std::vector<ORB_SLAM2::KeyFrame*> kfs_to_build_map_;

    std::vector<int> decisions_historic_;

    int exploration_state_;
    int past_exploration_state_;

    thread* thread_viewer_;
    thread* thread_itself_;

    float old_x, old_y, old_z, old_yaw;
    float uav_std_vel_;
    float uav_vel_mod_;

    //Parameters
    bool show_window_;
    int min_angle_turn_around_;
    float min_percent_unvisited_;
    int min_kfs_local_map_;
    float max_yaw_vel_;
    float max_vel_;
    float min_vel_;
    float uav_distance_to_voxel_;

};

#endif // EXPLORATION_H
