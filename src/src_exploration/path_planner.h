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

#ifndef PATHPLANNER_H
#define PATHPLANNER_H

#include <iostream>
#include <vector>

#include "System.h"
#include "src_exploration/local_map.h"
#include "src_exploration/util_configurations.h"

using namespace std;

struct PathStep{
    MapVoxel* voxel;
    float yaw;
    int step_number;
    bool finished;
};

class PathPlanner
{
public:
    PathPlanner(UtilConfigurations configuration, LocalMap* map);

    bool FindLocalPath(ORB_SLAM2::KeyFrame* kf_goal);

    vector<ORB_SLAM2::KeyFrame *> FindGlobalPath(ORB_SLAM2::KeyFrame* origin, ORB_SLAM2::KeyFrame* goal);

    bool IsInsideInflatedArea(MapVoxel* voxel);

    float CalculateCost(MapVoxel* father, MapVoxel* luke, int type_);
    double CalculateCost(cv::Mat position_a, cv::Mat position_b);
    double CalculateCost(vector<int> position_a, vector<int> position_b);

    int AngleAttraction(MapVoxel* origin_voxel, float yaw);

    vector<ORB_SLAM2::KeyFrame*> global_path();

    vector<PathStep> local_path();

    void SetStepFinished(int step_number);

    bool IsBetterRotateToRight(float min_rotation);

    enum DistanceTypes{
        EUCLIDEAN = 1,
        EUCLIDEAN_OBSTACLE_FREE = 2
    };

private:
    LocalMap* local_map_;

    vector<pair<MapVoxel*, float> > local_path_old_;

    vector<PathStep> local_path_;

    vector<ORB_SLAM2::KeyFrame*> global_path_;

    bool ready_;

    int write_code_;

    //Parameters
    float acceptable_distance_to_goal_;
    float free_cost_weight_;
    int free_cost_pad_;
    int inflate_obstacles_;
    int obstacles_multiplier_;
    float fov_horizontal_;
    float fov_vertical_;
    float camera_max_range_;
    float max_angle_rotation_;

};

#endif // PATHPLANNER_H
