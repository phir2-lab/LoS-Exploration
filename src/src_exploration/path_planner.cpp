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

#include <src_exploration/path_planner.h>

PathPlanner::PathPlanner(UtilConfigurations configuration, LocalMap* map)
{
    local_map_ = map;
    ready_ = true;

    write_code_ = 0;

    acceptable_distance_to_goal_ = configuration.GetInt("path_acceptable_distance_to_goal");

    free_cost_weight_ = configuration.GetFloat("path_free_cost_weight");
    free_cost_pad_ = configuration.GetInt("path_free_cost_pad");

    inflate_obstacles_ = configuration.GetInt("path_inflate_obstacles");
    obstacles_multiplier_ = configuration.GetInt("path_obstacles_multiplier");

    fov_horizontal_ = configuration.GetFloat("path_fov_horizontal");
    fov_vertical_ = configuration.GetFloat("path_fov_vertical");
    camera_max_range_ = configuration.GetFloat("path_camera_max_range");
    max_angle_rotation_ = configuration.GetFloat("path_max_angle_rotation");

    if(inflate_obstacles_ < 0) inflate_obstacles_ = 0;
}

//##############################
// FIND GLOBAL PATH
// Finds and return a path over the covisibility graph using A*
vector<ORB_SLAM2::KeyFrame*> PathPlanner::FindGlobalPath(ORB_SLAM2::KeyFrame *origin, ORB_SLAM2::KeyFrame *goal)
{
    ready_ = false;
    if(origin->mnId == goal->mnId)
    {
        global_path_.clear();
        global_path_.push_back(goal);
        return global_path_;
    }

    set<ORB_SLAM2::KeyFrame*> opened_kfs;
    set<ORB_SLAM2::KeyFrame*> closed_kfs;

    ORB_SLAM2::KeyFrame* actual = origin;
    actual->father = nullptr;
    actual->father_cost = 0;
    actual->total_cost = this->CalculateCost(actual->GetCameraCenter(), goal->GetCameraCenter());

    set<ORB_SLAM2::KeyFrame*> neighbors;
    while(actual)
    {
        // Checks if is the goal
        if(actual == goal)
        {
            goal->father = actual->father;
            break;
        }

        closed_kfs.insert(actual);
        opened_kfs.erase(actual);

        neighbors = actual->GetConnectedKeyFrames();

        for(set<ORB_SLAM2::KeyFrame*>::iterator it = neighbors.begin(); it != neighbors.end(); it++)
        {
            // Checks if is bad
            if((*it)->isBad()) continue;

            // Checks if is closed
            if(closed_kfs.count((*it))) continue;

            // Inserts in opened kfs
            if(opened_kfs.insert((*it)).second) // If is new
            {
                (*it)->father = actual;
                (*it)->father_cost = actual->father_cost + this->CalculateCost((*it)->GetCameraCenter(), actual->GetCameraCenter());
                (*it)->total_cost = (*it)->father_cost + this->CalculateCost((*it)->GetCameraCenter(), goal->GetCameraCenter());
            }
            else // If is old
            {
                double g = actual->father_cost + this->CalculateCost((*it)->GetCameraCenter(), actual->GetCameraCenter());
                if(g < (*it)->father_cost)
                {
                    (*it)->father = actual;
                    (*it)->father_cost = g;
                    (*it)->total_cost = g + this->CalculateCost((*it)->GetCameraCenter(), goal->GetCameraCenter());
                }
            }
        }

        // Gets the best candidate
        actual = nullptr;
        for(set<ORB_SLAM2::KeyFrame*>::iterator it = opened_kfs.begin(); it != opened_kfs.end(); it++)
        {
            if(actual == nullptr)
            {
                actual = (*it);
            }
            else if(actual->total_cost > (*it)->total_cost)
            {
                actual = (*it);
            }
        }
    }

    vector<ORB_SLAM2::KeyFrame*> path_aux;

    ORB_SLAM2::KeyFrame* step = goal;
    while(step)
    {
        path_aux.push_back(step);
        step = step->father;
    }

    global_path_ = path_aux;

    ready_ = true;
    return global_path_;
}

//##############################
// FIND LOCAL PATH
// Finds a path over the local map (voxels) using a modified version of A*
// Returns TRUE if exist a path or FALSE if not
bool PathPlanner::FindLocalPath(ORB_SLAM2::KeyFrame* kf_goal)
{
    ready_ = false;
    bool find_path = false;

    // Variables
    int map_lenght = local_map_->map_lenght();

    // Resets map and goal
    MapVoxel* goal = nullptr;
    for(int x=0; x<map_lenght;x++)
    {
        for(int y=0; y<map_lenght;y++)
        {
            for(int z=0; z<map_lenght;z++)
            {
                // A* variables reseted
                MapVoxel* voxel = local_map_->GetMapVoxel(x,y,z);
                voxel->astar_father(nullptr);
                voxel->astar_father_cost(-1);
                voxel->astar_supposed_cost(-1);
                voxel->astar_closed(false);

                if(voxel->IsGoal()) goal = local_map_->GetMapVoxel(x,y,z);
            }
        }
    }

    if(goal == nullptr)
    {
        ready_ = true;
        return false;
    }

    pair<MapVoxel, bool> uav_in_local_map = local_map_->uav_local_voxel();
    while(!uav_in_local_map.second)
    {
        uav_in_local_map = local_map_->uav_local_voxel();
    }

    vector<int> uav_position = uav_in_local_map.first.position_local();

    // Sets origin
    MapVoxel* start = local_map_->GetMapVoxel(uav_position);
    if(start == nullptr)
    {
        std::cout << "EXPLORATION: UAV outsite local map" << std::endl;
        ready_ = true;
        return false;
    }

    // Modified version of A* with weight paths (free + unknow)
    vector<MapVoxel*> candidates;

    MapVoxel* actual = start;
    actual->astar_father(nullptr);
    actual->astar_father_cost(0);
    actual->astar_closed(true);

    while(actual)
    {
        // If goal
        if(CalculateCost(actual, goal, PathPlanner::EUCLIDEAN) <= acceptable_distance_to_goal_ && kf_goal == nullptr)
        {
            // Checks goal
            if(actual->astar_father_cost() < goal->astar_father_cost() || goal->astar_father() == nullptr)
            {
                goal->astar_father_cost(actual->astar_father_cost());
                goal->astar_father(actual);
            }
        }
        else if(CalculateCost(actual, goal, PathPlanner::EUCLIDEAN) == 0 && kf_goal != nullptr)
        {
            // Checks goal
            if(actual->astar_father_cost() < goal->astar_father_cost() || goal->astar_father() == nullptr)
            {
                goal->astar_father_cost(actual->astar_father_cost());
                goal->astar_father(actual);
            }
        }

        // Gets new candidates
        for(int x=-1; x<=1; x++)
        {
            for(int y=-1; y<=1; y++)
            {
                for(int z=-1; z<=1; z++)
                {
                    // If it is not the actual
                    if(x != 0 || y != 0 || z != 0)
                    {
                        MapVoxel* voxel = local_map_->GetMapVoxel(actual->position_local()[0]+x, actual->position_local()[1]+y, actual->position_local()[2]+z);

                        if(voxel == nullptr) continue; // If outside of map

                        if(voxel->IsFree() && !voxel->astar_closed() && !this->IsInsideInflatedArea(voxel))
                        {
                            // If new in the candidates
                            if(voxel->astar_father() == nullptr)
                            {
                                float cost = CalculateCost(actual, voxel, PathPlanner::EUCLIDEAN_OBSTACLE_FREE);

                                voxel->astar_father_cost(actual->astar_father_cost() + cost);
                                voxel->astar_father(actual);
                                voxel->astar_supposed_cost(CalculateCost(voxel, goal, PathPlanner::EUCLIDEAN));
                                voxel->astar_total_cost(voxel->astar_father_cost() + voxel->astar_supposed_cost());
                                candidates.push_back(voxel);
                            }
                            else
                            {
                                float cost = CalculateCost(actual, voxel, PathPlanner::EUCLIDEAN_OBSTACLE_FREE);

                                // If this way is better than the old way
                                if(actual->astar_father_cost() + cost < voxel->astar_father_cost())
                                {
                                    voxel->astar_father_cost(actual->astar_father_cost() + cost);
                                    voxel->astar_father(actual);
                                    voxel->astar_total_cost(voxel->astar_father_cost() + voxel->astar_supposed_cost());
                                }
                            }
                        }
                    }
                }
            }
        }// End fors

        int best = -1;
        float best_cost = -1;

        for(size_t i=0; i<candidates.size(); i++)
        {
            if(candidates[i]->astar_total_cost() < best_cost || best == -1)
            {
                best = i;
                best_cost = candidates[i]->astar_total_cost();
            }
        }

        if(candidates.empty())
            actual = nullptr;
        else
        {
            actual = candidates[best];
            actual->astar_closed(true);
            candidates.erase(candidates.begin() + best);
        }

    } // End of while

    vector<MapVoxel*> path_aux;

    MapVoxel* step = goal->astar_father();
    while(step)
    {
        find_path = true;
        path_aux.insert(path_aux.begin(), step);
        if(step->astar_father() != nullptr)
            if(CalculateCost(step->astar_father(), step, PathPlanner::EUCLIDEAN) == 0)
                break;
        step = step->astar_father();
    }

    // Defines the yaw angle for each step
    if(find_path)
    {
        local_path_.clear();

        if(path_aux.size() > 0)
        {

            float uav_yaw = local_map_->uav_yaw()*180/M_PI; // deg

            float target_angle;

            if(kf_goal != nullptr)
                target_angle = local_map_->get_kf_yaw(kf_goal->GetPose())*180/M_PI;
            else
            {
                MapVoxel* final_step = goal->astar_father();

                target_angle = uav_yaw;
                float best_score = this->AngleAttraction(final_step, uav_yaw);

                float aux_angle;
                float score;
                for(int c=1; c < path_aux.size(); c++)
                {
                    aux_angle = uav_yaw + max_angle_rotation_*c;
                    aux_angle += (aux_angle > 180) ? -360 : (aux_angle < -180) ? 360 : 0;
                    score = this->AngleAttraction(final_step, aux_angle);
                    if(score > best_score)
                    {
                        target_angle = aux_angle;
                        best_score = score;
                    }

                    aux_angle = uav_yaw - max_angle_rotation_*c;
                    aux_angle += (aux_angle > 180) ? -360 : (aux_angle < -180) ? 360 : 0;
                    score = this->AngleAttraction(final_step, aux_angle);
                    if(score > best_score)
                    {
                        target_angle = aux_angle;
                        best_score = score;
                    }
                }
            }

            float yaw_step = 0;
            yaw_step = target_angle-uav_yaw;
            yaw_step += (yaw_step > 180) ? -360 : (yaw_step < -180) ? 360 : 0;
            yaw_step = yaw_step/(path_aux.size()-1);

            for(uint i=0; i<path_aux.size(); i++)
            {
                local_path_.push_back(PathStep());
                local_path_.back().voxel = path_aux[i];
                local_path_.back().yaw = uav_yaw;
                local_path_.back().step_number = i;
                if(i != 0)
                    local_path_.back().finished = false;
                else
                    local_path_.back().finished = true;
                uav_yaw += yaw_step;
            }

        }
    }

    ready_ = true;

    if(find_path) return true;
    return false;
}

//##############################
// IS BETTER ROTATE TO RIGH
// Defines if is better to rotate to right or left
bool PathPlanner::IsBetterRotateToRight(float min_rotation)
{
    float uav_yaw = local_map_->uav_yaw()*180/M_PI; // deg
    pair<MapVoxel, bool> uav_voxel = local_map_->uav_local_voxel();

    if(uav_voxel.second)
    {
        if(this->AngleAttraction(&uav_voxel.first, uav_yaw+min_rotation) > this->AngleAttraction(&uav_voxel.first, uav_yaw-min_rotation))
            return true;
        else return false;
    }

    return true;
}

//##############################
// ANGLE ATTRACTION
// Calculates the Expected Observable Space given a pose
int PathPlanner::AngleAttraction(MapVoxel* origin_voxel, float yaw)
{
    write_code_++;

    int score = 0;

    float half_fov_h = fov_horizontal_/2;
    float half_fov_v = fov_vertical_/2;

    // DDA variables
    float dx, dy, dz, steps, x_increment, y_increment, z_increment;

    for(int f_h = -half_fov_h; f_h <= half_fov_h; f_h++)
    {
        for(int f_v = -half_fov_v; f_v <= half_fov_v; f_v++)
        {

            vector<int> point = origin_voxel->position_local();

            float longitude_angle = (yaw + f_h)*M_PI/180;
            float latitude_angle = (f_v)*M_PI/180;

            float end_point[3];
            end_point[2] = point[2] + camera_max_range_ * cos(latitude_angle) * cos(longitude_angle); // X VISUAL
            end_point[0] = point[0] + camera_max_range_ * cos(latitude_angle) * sin(longitude_angle); // Y VISUAL
            end_point[1] = point[1] + camera_max_range_ * sin(latitude_angle); // Z VISUAL

            // Uses DDA to calculate the line of sight
            dx = end_point[0] - point[0];
            dy = end_point[1] - point[1];
            dz = end_point[2] - point[2];

            steps = std::max(abs(dx), abs(dy));
            steps = std::max(steps, abs(dz));

            steps = ceil(steps);

            // Determines the increments
            x_increment = dx/steps;
            y_increment = dy/steps;
            z_increment = dz/steps;

            // Get the base point
            float pointer[3];
            pointer[0] = point[0];
            pointer[1] = point[1];
            pointer[2] = point[2];

            for(int count = 1; count<=steps; count++) // Iterates over steps
            {
                MapVoxel* voxel = local_map_->GetMapVoxel(round(pointer[0]),  round(pointer[1]), round(pointer[2]));

                if(voxel == nullptr)
                {
                    score++;
                    continue;
                }

                if(voxel->IsObstacle()) break;

                if(voxel->UpdatePlanningCode(write_code_))
                    score++;

                // Increment the base point
                pointer[0] = pointer[0] + x_increment;
                pointer[1] = pointer[1] + y_increment;
                pointer[2] = pointer[2] + z_increment;
            }
        }
    }

    return score;
}

//##############################
// IS INSIDE INFLATED AREA
// Evaluates if a voxel is inside a inflated area around the obstacles
bool PathPlanner::IsInsideInflatedArea(MapVoxel *voxel)
{
    for(int x=-inflate_obstacles_; x<=inflate_obstacles_; x++)
    {
        for(int y=-inflate_obstacles_; y<=inflate_obstacles_; y++)
        {
            for(int z=-inflate_obstacles_; z<=inflate_obstacles_; z++)
            {
                MapVoxel *voxel_aux = local_map_->GetMapVoxel(voxel->position_local()[0]+x, voxel->position_local()[1]+y, voxel->position_local()[2]+z);
                if(voxel_aux != nullptr)
                    if(voxel_aux->IsObstacle()) return true;
            }
        }

    }

    return false;
}

//##############################
// CALCULATE COST
// Calculates the cost between two local map voxels
float PathPlanner::CalculateCost(MapVoxel *father, MapVoxel *luke, int type)
{
    float cost = 0;

    // Just distance
    if(type == PathPlanner::EUCLIDEAN)
    {
        cost = sqrt(pow((father->position_local()[0]-luke->position_local()[0]),2) + pow((father->position_local()[1]-luke->position_local()[1]),2) + pow((father->position_local()[2]-luke->position_local()[2]),2));
    }

    // Model with distance + free/obstacle
    else if(type == PathPlanner::EUCLIDEAN_OBSTACLE_FREE)
    {

        // Distance
        float distance_cost = sqrt(pow((father->position_local()[0]-luke->position_local()[0]),2) + pow((father->position_local()[1]-luke->position_local()[1]),2) + pow((father->position_local()[2]-luke->position_local()[2]),2));

        // Free rate
        MapVoxel *voxel;
        float dist;
        float free_cost = 0;

        float helper;

        for(int x=-free_cost_pad_; x<=free_cost_pad_; x++)
        {
            for(int y=-free_cost_pad_; y<=free_cost_pad_; y++)
            {
                for(int z=-free_cost_pad_; z<=free_cost_pad_; z++)
                {
                    voxel = local_map_->GetMapVoxel(luke->position_local()[0]+x, luke->position_local()[1]+y, luke->position_local()[2]+z);
                    if(voxel)
                    {
                        dist = 1 + sqrt(pow(x,2) + pow(y,2) + pow(z,2));

                        if(voxel->IsObstacle())
                            free_cost += obstacles_multiplier_*voxel->free_rate_max()/dist;

                        else if(voxel->IsUnknow())
                            free_cost += voxel->free_rate_max()/dist;

                        else
                            free_cost += (voxel->free_rate_max() - voxel->free_rate())/dist;

                    }
                }
            }
        }

        cost = distance_cost + free_cost*free_cost_weight_;
    }

    return cost;
}

//##############################
// CALCULATE COST
// Calculates the cost between two positions using euclidean distance
double PathPlanner::CalculateCost(cv::Mat position_a, cv::Mat position_b)
{
    return sqrt(pow((position_a.at<float>(0)-position_b.at<float>(0)),2) + pow((position_a.at<float>(1)-position_b.at<float>(1)),2) + pow((position_a.at<float>(2)-position_b.at<float>(2)),2));
}

//##############################
// CALCULATE COST
// Calculates the cost between two the positions using euclidean distance
double PathPlanner::CalculateCost(vector<int> position_a, vector<int> position_b)
{
    return sqrt(pow((position_a[0]-position_b[0]),2) + pow((position_a[1]-position_b[1]),2) + pow((position_a[2]-position_b[2]),2));
}

vector<PathStep> PathPlanner::local_path()
{
    return local_path_;
}

void PathPlanner::SetStepFinished(int step_number)
{
    local_path_[step_number].finished = true;
}

vector<ORB_SLAM2::KeyFrame *> PathPlanner::global_path()
{
    return global_path_;
}