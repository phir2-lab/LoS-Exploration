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

#ifndef MAPVOXEL_H
#define MAPVOXEL_H

#include <mutex>
#include <vector>

#include "System.h"
#include "src_exploration/util_configurations.h"

using namespace std;

class MapVoxel
{
public:
    MapVoxel();

    void SetParameters(UtilConfigurations configuration);

    float free_rate();
    float free_rate_max();

    bool AddFreeObservation(int code);

    void AddAttraction(float a);
    void MeasureAttractionByDistance(float distance);
    float attraction();

    bool IsFree();
    bool IsObstacle();
    bool IsGoal();
    bool IsUnknow();
    bool IsInteresting(); //Checks if the voxel is interesting

    void TurnToFree();

    void EnableGoal();
    void DisableGoal();

    void Clear();

    vector<int> position_local();
    void position_local(int x, int y, int z);

    vector<float> position_global();
    void position_global(float x, float y, float z);

    set<ORB_SLAM2::MapPoint *> points();
    bool AddPoint(ORB_SLAM2::MapPoint *point);
    void ErasePoint(ORB_SLAM2::MapPoint *point);

    //A* functions
    MapVoxel *astar_father();
    void astar_father(MapVoxel *value);
    float astar_father_cost();
    void astar_father_cost(float value);
    float astar_supposed_cost();
    void astar_supposed_cost(float value);
    float astar_total_cost();
    void astar_total_cost(float value);
    bool astar_closed();
    void astar_closed(bool value);

    bool UpdatePlanningCode(int value);

private:
    static std::mutex position_mutex_;

    set<ORB_SLAM2::MapPoint*> points_;

    vector<int> position_local_;
    vector<float> position_global_;

    int path_planning_code_;

    float attraction_;
    float lambda_;

    int free_write_code_;
    float free_rate_;

    bool is_a_goal_;
    bool is_uav_position_;
    int part_of_path_;

    //A*
    MapVoxel* astar_father_;
    float astar_father_cost_;
    float astar_supposed_cost_;
    float astar_total_cost_;
    bool astar_closed_;

    //Parameters
    float free_rate_threshold_;
    float free_rate_delta_;
    float free_rate_max_;
    float obstacle_rate_threshold_;
    int min_points_;
    float min_rate_between_visited_unvisited_;

};

#endif // MapVoxel_H
