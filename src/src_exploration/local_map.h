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

#ifndef LOCALMAP_H
#define LOCALMAP_H

#include <math.h>

#include <opencv2/core/core.hpp>

#include "System.h"
#include "Converter.h"
#include "src_exploration/map_voxel.h"
#include "src_exploration/util_configurations.h"

using namespace std;

class LocalMap
{
public:
    LocalMap(UtilConfigurations configuration);

    void Reset();
    void ClearMap();

    vector<int> ConvertFromGlobalToLocal(cv::Mat point_pose, cv::Mat reference_pose, float scale, float pad); //Converts the global position to local postion

    bool SetUavPositions(cv::Mat value);

    void SetUavPoseMat(cv::Mat value);

    float uav_yaw();

    bool IsInsideMap(int x, int y, int z); //Checks if the coordinate is inside the map
    bool IsInsideMap(vector<int> p); //Checks if the coordinate is inside the map

    MapVoxel* GetMapVoxel(int x, int y, int z);
    MapVoxel* GetMapVoxel(vector<int> p);
    float voxel_size();

    void SetVoxelAsFree(vector<int> position);
    void AddVoxelFreeObservation(vector<int> position, int write_code_);
    void AddVoxelObstacleObservation(vector<int> position, ORB_SLAM2::MapPoint* point);

    void UpdateLocalMapCode();

    void UpdateWriteCode();
    void UpdateMapFromKf(ORB_SLAM2::KeyFrame *kfs); //Updates local map with informations from one ketframe (line of sight)
    void UpdateMapDuringNavigation(vector<ORB_SLAM2::KeyFrame *> kfs); //Updates local map with informations from a vector of keyframes (just the new obstacles)

    void SetGoal(vector<int> p);

    bool DefineNextGoal(); //Searches a new goal
	
    void CheckReachedPoints(); //Sets to visited the unvisited closer points

    void GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M);

    double get_kf_yaw(cv::Mat value);

    MapVoxel* goal_local_voxel();
    void EraseGoal();

    void ClearOutsidePoints(MapVoxel *voxel);

    ORB_SLAM2::KeyFrame *reference_kf();
    void reference_kf(ORB_SLAM2::KeyFrame *reference_kf);

    void ComputeGlobalPositionOfVoxels();

    void ChangeScale(std::vector<std::pair<float, float>> historic);
    float grid_scale();
    float real_scale();

    int map_lenght();
    int pad_half_map();

    pair<MapVoxel, bool> uav_local_voxel();

    std::set<MapVoxel *> voxels_obstacle();
    std::set<MapVoxel *> voxels_free();

    static float EuclidianDistance(vector<float> p1, vector<float> p2);
    static float EuclidianDistance(vector<int> p1, vector<int> p2);
    static float EuclidianDistance(float p1x, float p1y, float p1z, float p2x, float p2y, float p2z);

    static std::mutex map_mutex;

private:
    MapVoxel*** map_;

    std::set<MapVoxel*> voxels_obstacle_;
    std::set<MapVoxel*> voxels_free_;

    MapVoxel* uav_local_voxel_;
    MapVoxel* goal_local_voxel_;

    std::vector<ORB_SLAM2::MapPoint*> candidates_to_goal_;
    ORB_SLAM2::KeyFrame* reference_kf_;

    bool last_direction_right_;

    int biggest_attraction_;

    float real_scale_;
    float grid_scale_;
    float voxel_size_;
    float real_voxel_size_;
    int pad_half_map_;

    cv::Mat uav_global_position_;
    cv::Mat uav_global_pose_mat_;
    float uav_yaw_;

    int write_code_;
    int local_map_code_;

    //Parameters
    int acceptable_distance_to_goal_;
    int map_lenght_;
    float free_rate_threshold_;
    float obstacle_rate_threshold_;

};

#endif // LOCALMAP_H
