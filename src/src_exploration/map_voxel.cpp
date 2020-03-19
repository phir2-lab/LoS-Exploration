/**
 * This file is part of LoS-Exploration.
 *
 * Copyright 2020 Diego Pittol <dpittol at inf dot ufrgs dot br> (Phi Robotics Research Lab - UFRGS)
 * For more information see <https:// github.com/phir2-lab/LoS-Exploration>
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
 * along with LoS-Exploration. If not, see <https:// www.gnu.org/licenses/>.
**/

#include <src_exploration/map_voxel.h>

mutex MapVoxel::position_mutex_;

MapVoxel::MapVoxel(){
    path_planning_code_ = -1;
    free_write_code_ = -1;
    attraction_ = 0;

    free_rate_ = 0;

    is_a_goal_ = false;

    part_of_path_=0;

    is_uav_position_=false;
}

void MapVoxel::SetParameters(UtilConfigurations configuration)
{
    lambda_ = configuration.GetFloat("goal_distance_lambda");

    free_rate_delta_ = configuration.GetFloat("voxel_free_rate_delta");
    free_rate_max_ = configuration.GetFloat("voxel_free_rate_max");
    free_rate_threshold_ = configuration.GetFloat("voxel_free_rate_threshold");

    obstacle_rate_threshold_ = configuration.GetFloat("voxel_obstacle_rate_threshold");

    min_points_ = configuration.GetInt("voxel_min_points");
    min_rate_between_visited_unvisited_ = configuration.GetFloat("voxel_min_percent_unvisited");
}

float MapVoxel::free_rate()
{
    return free_rate_;
}

float MapVoxel::free_rate_max()
{
    return free_rate_max_;
}

bool MapVoxel::AddFreeObservation(int code)
{
    if(code > free_write_code_){
        free_write_code_ = code;
        free_rate_ += free_rate_delta_;
        if(free_rate_ > free_rate_max_) free_rate_ = free_rate_max_;
    }

    return this->IsFree();
}

bool MapVoxel::IsFree()
{
    if(points_.size() > obstacle_rate_threshold_ || free_rate_ < free_rate_threshold_) return false;

    return true;
}

bool MapVoxel::IsObstacle()
{
    if(points_.size() > obstacle_rate_threshold_) return true;
    else return false;
}

bool MapVoxel::IsGoal()
{
    return is_a_goal_;
}

bool MapVoxel::IsUnknow()
{
    if(this->IsObstacle()) return false;
    if(this->IsFree()) return false;
    return true;
}

void MapVoxel::EnableGoal()
{
    is_a_goal_ = true;
}

void MapVoxel::DisableGoal()
{
    is_a_goal_ = false;
}

//##############################
// IS INTERESTING
// Checks if the voxel is interesting with two measures
// - The voxel has a sufficient points unvisited?
// - The voxel has more points unvisited that points visited?
bool MapVoxel::IsInteresting()
{
    int count_unvisited = 0;
    int count_visited = 0;

    for(set<ORB_SLAM2::MapPoint*>::iterator it = points_.begin(); it != points_.end(); it++)
    {
        if((*it)->reached() == false) count_unvisited++;
        else count_visited++;
    }

    if(float(count_unvisited)/points_.size() >= min_rate_between_visited_unvisited_ && count_unvisited >= min_points_)
    {
        return true;
    }
    else if(count_visited >= min_points_) {
        for(set<ORB_SLAM2::MapPoint*>::iterator it = points_.begin(); it != points_.end(); it++)
        {
            (*it)->reached(true);
        }
        return false;
    }
    return false;
}

void MapVoxel::TurnToFree()
{
    free_rate_ = 1;
}

void MapVoxel::AddAttraction(float a)
{
    attraction_ += 1.0-(1.0/a);
}

void MapVoxel::MeasureAttractionByDistance(float distance)
{
    if(attraction_ == 0) return;

    attraction_ *= exp(-lambda_*distance);
}

float MapVoxel::attraction()
{
    return attraction_;
}

void MapVoxel::Clear()
{
    free_rate_ = 0;
    is_a_goal_ = false;
    attraction_ = 0;

    part_of_path_=0;

    // Clear the keyframes observation counter of points
    for(set<ORB_SLAM2::MapPoint*>::iterator it = points_.begin(); it != points_.end(); it++)
        (*it)->observations_from_kf(0);

    points_.clear();
}

void MapVoxel::position_local(int x, int y, int z)
{
    unique_lock<mutex> lock(position_mutex_);
    position_local_.clear();
    position_local_.push_back(x);
    position_local_.push_back(y);
    position_local_.push_back(z);
}

vector<int> MapVoxel::position_local()
{
    unique_lock<mutex> lock(position_mutex_);
    return position_local_;
}

vector<float> MapVoxel::position_global()
{
    return position_global_;
}

void MapVoxel::position_global(float x, float y, float z)
{
    position_global_.clear();
    position_global_.push_back(x);
    position_global_.push_back(y);
    position_global_.push_back(z);
}

set<ORB_SLAM2::MapPoint *> MapVoxel::points()
{
    return points_;
}

bool MapVoxel::AddPoint(ORB_SLAM2::MapPoint *point)
{
    points_.insert(point);
    return this->IsObstacle();
}

void MapVoxel::ErasePoint(ORB_SLAM2::MapPoint* point)
{
    points_.erase(point);
}

MapVoxel *MapVoxel::astar_father()
{
    return astar_father_;
}

bool MapVoxel::UpdatePlanningCode(int value)
{
    if(path_planning_code_ != value)
    {
        path_planning_code_ = value;
        return true;
    }
    return false;
}

void MapVoxel::astar_father(MapVoxel *value)
{
    astar_father_ = value;
}

float MapVoxel::astar_father_cost()
{
    return astar_father_cost_;
}

void MapVoxel::astar_father_cost(float value)
{
    astar_father_cost_ = value;
}

float MapVoxel::astar_supposed_cost()
{
    return astar_supposed_cost_;
}

void MapVoxel::astar_supposed_cost(float value)
{
    astar_supposed_cost_ = value;
}

float MapVoxel::astar_total_cost()
{
    return astar_total_cost_;
}

void MapVoxel::astar_total_cost(float value)
{
    astar_total_cost_ = value;
}

bool MapVoxel::astar_closed()
{
    return astar_closed_;
}

void MapVoxel::astar_closed(bool value)
{
    astar_closed_ = value;
}