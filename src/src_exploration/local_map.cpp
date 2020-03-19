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

#include <src_exploration/local_map.h>

mutex LocalMap::map_mutex;

LocalMap::LocalMap(UtilConfigurations configuration)
{

    write_code_ = 0; // Initializes write code
    local_map_code_ = 0;

    last_direction_right_ = true;

    acceptable_distance_to_goal_ = configuration.GetInt("path_acceptable_distance_to_goal");

    real_voxel_size_ = configuration.GetFloat("map_voxel_size");

    voxel_size_ = 0.1;
    grid_scale_ = 1/voxel_size_;
    map_lenght_ = configuration.GetInt("local_map_size");
    pad_half_map_ = map_lenght_/2;

	// Initializes local map
    map_ = new MapVoxel**[map_lenght_]; 
    for(int i =0; i<map_lenght_; i++)
	{
       map_[i] = new MapVoxel*[map_lenght_];
       for(int j =0; j<map_lenght_; j++)
	   {
           map_[i][j] = new MapVoxel[map_lenght_];
       }
    }

    reference_kf_ = nullptr;
    goal_local_voxel = nullptr;
    uav_local_voxel_ = nullptr;

    for(int x=0;x<map_lenght_;x++)
	{
        for(int y=0;y<map_lenght_;y++)
		{
            for(int z=0;z<map_lenght_;z++)
			{
                // Sets configuration parameters
                map_[x][y][z].SetParameters(configuration);
                // Initializes local position
                map_[x][y][z].position_local(x, y, z);
            }
        }
    }
}

//##############################
// SET UAV POSITIONS
// Sets uav global position and uav local position
// Returns FALSE if uav is outside of the local map
bool LocalMap::SetUavPositions(cv::Mat value)
{
    unique_lock<mutex> lock(map_mutex);
    uav_global_position_ = value;

    if(reference_kf_ != nullptr)
    {
        cv::Mat reference_position = reference_kf_->GetCameraCenter();

        vector<int> point_coordinates = this->ConvertFromGlobalToLocal(uav_global_position_, reference_position, grid_scale_, pad_half_map_);

        if(!IsInsideMap(point_coordinates)) return false;

        uav_local_voxel_ = this->GetMapVoxel(point_coordinates);
    }

    return true;
}

//##############################
// DEFINE NEXT GOAL
// Defines and returns the new goal. If haven't, returns false.
bool LocalMap::DefineNextGoal()
{
    unique_lock<mutex> lock(map_mutex);

    int best_x, best_y, best_z;
    best_x=best_y=best_z=0;

    biggest_attraction_ = -1;

    // Passes over the local map
    set<ORB_SLAM2::MapPoint*> voxel_points_set;
    for(int x=0;x<map_lenght_;x++)
    {
        for(int y=0;y<map_lenght_;y++)
        {
            for(int z=0;z<map_lenght_;z++)
            {
                // Gets the points associated with voxel                
                this->ClearOutsidePoints(this->GetMapVoxel(x, y, z));

                if(!map_[x][y][z].IsInteresting()) continue;

                voxel_points_set = map_[x][y][z].points();

                // Passes over the voxels's points
                for(set<ORB_SLAM2::MapPoint*>::iterator it = voxel_points_set.begin(); it != voxel_points_set.end(); it++)
                {
                    // Adds attraction to the voxel if the point wasn't reached
                    if(!(*it)->reached())
                        map_[x][y][z].AddAttraction((*it)->observations_from_kf());
                }

                map_[x][y][z].MeasureAttractionByDistance(EuclidianDistance(uav_local_voxel_->position_local(), map_[x][y][z].position_local()));

                if(map_[x][y][z].attraction() > biggest_attraction_){
                    best_x = x;
                    best_y = y;
                    best_z = z;
                    biggest_attraction_ = map_[x][y][z].attraction();
                }
            }
        }
    }

    goal_local_voxel_ = nullptr;

    if(biggest_attraction_ > 0)
    {
        map_[best_x][best_y][best_z].EnableGoal();
        goal_local_voxel_ = this->GetMapVoxel(best_x, best_y, best_z);
        return true;
    }

    return false;
}

//##############################
// SET GOAL
// Disable previously goal and set new
void LocalMap::SetGoal(vector<int> p)
{
    if(goal_local_voxel_)
        if(this->IsInsideMap(goal_local_voxel_->position_local()))
            goal_local_voxel_->DisableGoal();
    goal_local_voxel_ = this->GetMapVoxel(p);
    goal_local_voxel_->EnableGoal();
}

//##############################
// CHECK REACHED POINTS
// Sets to visited the unvisited closer points
void LocalMap::CheckReachedPoints()
{
    unique_lock<mutex> lock(map_mutex);

    vector<int> uav_local_position = uav_local_voxel_->position_local();

    MapVoxel* voxel_aux;
    set<ORB_SLAM2::MapPoint*> set_of_points;
    for(int pad_x = -acceptable_distance_to_goal_; pad_x<=acceptable_distance_to_goal_; pad_x++)
    {
        for(int pad_y = -acceptable_distance_to_goal_; pad_y<=acceptable_distance_to_goal_; pad_y++)
        {
            for(int pad_z = -acceptable_distance_to_goal_; pad_z<=acceptable_distance_to_goal_; pad_z++)
            {
                if(IsInsideMap(uav_local_position[0]+pad_x, uav_local_position[1]+pad_y, uav_local_position[2]+pad_z))
                {
                    // Calculates the Euclidian distance
                    if(acceptable_distance_to_goal_ >= sqrt(pow(pad_x,2) + pow(pad_y,2) + pow(pad_z,2)))
                    {
                        voxel_aux = this->GetMapVoxel(uav_local_position[0]+pad_x, uav_local_position[1]+pad_y, uav_local_position[2]+pad_z);
                        set_of_points = voxel_aux->points();
                        for(set<ORB_SLAM2::MapPoint*>::iterator it = set_of_points.begin(); it != set_of_points.end(); it++)
                        {
                            (*it)->reached(true);
                        }
                    }
                }
            }
        }
    }
}

//##############################
// ADD VOXEL OBSTACLE OBSERVATION
// Add an obstacle observation and the point to the voxel' set of points
// Add an keyframe observation to the point
void LocalMap::AddVoxelObstacleObservation(vector<int> position, ORB_SLAM2::MapPoint *point)
{
    if(IsInsideMap(position))
    {
        MapVoxel* voxel = this->GetMapVoxel(position);
        if(voxel->AddPoint(point))
            voxels_obstacle_.insert(voxel);
        else
            voxels_obstacle_.erase(voxel);

        if(!voxel->IsFree())
            voxels_free_.erase(voxel);

        point->AddObservationsFromKf(local_map_code_);
    }
}

//##############################
// ADD VOXEL FREE OBSERVATION
// Add an free observation to the voxel
void LocalMap::AddVoxelFreeObservation(vector<int> position, int write_code_)
{
    if(IsInsideMap(position))
    {
        MapVoxel* voxel = this->GetMapVoxel(position);

        if(voxel->AddFreeObservation(write_code_))
            voxels_free_.insert(voxel);
        else
            voxels_free_.erase(voxel);
    }
}

//##############################
// UPDATE MAP FROM KEYFRAME
// Updates the local map using information provide by a keyframe
void LocalMap::UpdateMapFromKf(ORB_SLAM2::KeyFrame* kfs)
{
    this->UpdateWriteCode();
    unique_lock<mutex> lock(map_mutex);
	
    cv::Mat reference_position = reference_kf_->GetCameraCenter(); // Gets the local map reference position in global map

    set<ORB_SLAM2::MapPoint*> points_set = kfs->GetMapPoints(); // Gets the keyframe's points
    cv::Mat kf_position = kfs->GetCameraCenter(); // Gets the keyframe's global position

    // Sets as free the voxel corresponding to the keyframe's local pose
    this->SetVoxelAsFree(this->ConvertFromGlobalToLocal(kf_position, reference_position, grid_scale_, pad_half_map_));

    // Iterates over keyframe's points
    cv::Mat point_position;
    cv::Mat point = cv::Mat(1, 3, CV_32F);
    float dx, dy, dz, steps, x_increment, y_increment, z_increment;
    for (set<ORB_SLAM2::MapPoint*>::iterator it = points_set.begin(); it != points_set.end(); it++)
    {
        if((*it)->isBad()) continue;

        point_position = (*it)->GetWorldPos();

        this->AddVoxelObstacleObservation(this->ConvertFromGlobalToLocal(point_position, reference_position, grid_scale_, pad_half_map_), (*it));

        // Uses DDA to calculate the line of sight
        dx = point_position.at<float>(0) - kf_position.at<float>(0);
        dy = point_position.at<float>(1) - kf_position.at<float>(1);
        dz = point_position.at<float>(2) - kf_position.at<float>(2);

        if(abs(dx) >= abs(dy))
        {
            if(abs(dx) > abs(dz)) steps = abs(dx)/voxel_size_;
            else steps = abs(dz)/voxel_size_;
        }
        else if(abs(dy) > abs(dz)) steps = abs(dy)/voxel_size_;
        else steps = abs(dz)/voxel_size_;

        steps = ceil(steps);

        // Determines the increments
        x_increment = dx/steps;
        y_increment = dy/steps;
        z_increment = dz/steps;

        // Get the base point
        point.at<float>(0) = kf_position.at<float>(0);
        point.at<float>(1) = kf_position.at<float>(1);
        point.at<float>(2) = kf_position.at<float>(2);

        for(int count = 1; count<=steps; count++)
        {
            // Increment the base point
            point.at<float>(0) = point.at<float>(0) + x_increment;
            point.at<float>(1) = point.at<float>(1) + y_increment;
            point.at<float>(2) = point.at<float>(2) + z_increment;

            this->AddVoxelFreeObservation(this->ConvertFromGlobalToLocal(point, reference_position, grid_scale_, pad_half_map_), write_code_);
        }
    }
}

//##############################
// UPDATE MAP DURING NAVIGATION
// Updates the map with new points during the navigation state
void LocalMap::UpdateMapDuringNavigation(vector<ORB_SLAM2::KeyFrame *> kfs)
{
    if(kfs.empty()) return;

    UpdateWriteCode();

    unique_lock<mutex> lock(map_mutex);
    cv::Mat reference_position = reference_kf_->GetCameraCenter();

    // Passes over the other KFs
    set<ORB_SLAM2::MapPoint*> points_set;
    cv::Mat point_position;
    vector<int> point_coordinates;
    for(size_t i=0; i<kfs.size(); i++)
    {
        // Gets the keyframe's points
        points_set = kfs[i]->GetMapPoints();

        // Passes over the points to check new points
        for (set<ORB_SLAM2::MapPoint*>::iterator it = points_set.begin(); it != points_set.end(); it++)
        {
            if((*it)->isBad()) continue;
            if((*it)->code_ == local_map_code_) continue; // Already processed

            point_position = (*it)->GetWorldPos();
            point_coordinates = this->ConvertFromGlobalToLocal(point_position, reference_position, grid_scale_, pad_half_map_);

            if(IsInsideMap(point_coordinates))
                map_[point_coordinates[0]][point_coordinates[1]][point_coordinates[2]].AddPoint((*it));
        }
    }
}

//##############################
// CONVERT FROM GLOBAL TO LOCAL
// Converts from global position to a local position
vector<int> LocalMap::ConvertFromGlobalToLocal(cv::Mat point_pose, cv::Mat reference_position, float scale, float pad)
{
    int pointX = floor(scale * (point_pose.at<float>(0) - reference_position.at<float>(0)));
    int pointY = floor(scale * (point_pose.at<float>(1) - reference_position.at<float>(1)));
    int pointZ = floor(scale * (point_pose.at<float>(2) - reference_position.at<float>(2)));

    vector<int> point;

    point.push_back(pointX + pad);
    point.push_back(pointY + pad);
    point.push_back(pointZ + pad);

    return point;
}

void LocalMap::ClearOutsidePoints(MapVoxel* voxel)
{
    // Passes over the points to check if continues inside voxel
    cv::Mat point_position;
    vector<int> point_coordinates;

    set<ORB_SLAM2::MapPoint*>::iterator temp_it;

    set<ORB_SLAM2::MapPoint*> points = voxel->points();

    for (set<ORB_SLAM2::MapPoint*>::iterator it = points.begin(); it != points.end(); ++it)
    {

        if((*it)->isBad())
        {
            temp_it = it++;
            points.erase((*it));
            voxel->ErasePoint((*it));
            it = temp_it;
            continue;
        }

        point_position = (*it)->GetWorldPos();

        point_coordinates = this->ConvertFromGlobalToLocal(point_position, reference_kf_->GetCameraCenter(), grid_scale_, pad_half_map_);

        if(point_coordinates[0] != voxel->position_local()[0] || point_coordinates[1] != voxel->position_local()[1] || point_coordinates[2] != voxel->position_local()[2])
        {
            temp_it = it++;
            points.erase((*it));
            voxel->ErasePoint((*it));
            it = temp_it;
        }
    }
}

void LocalMap::ChangeScale(std::vector<std::pair<float, float> > historic)
{
    float sum_scales = 0;

    unique_lock<mutex> lock(map_mutex);

    for(int i=0; i < (int)historic.size(); i++)
    {
        // first = real, second = slam
        sum_scales += historic[i].second/historic[i].first;
    }

    real_scale_ = sum_scales/historic.size();

    voxel_size_ = real_voxel_size_ * real_scale_;

    grid_scale_ = 1/voxel_size_;

    std::cout << fixed << setprecision(4) << "EXPLORATION: " << "Scale: " << real_scale_ << std::endl; //  " | Real: " << historic.back().first << " | Slam: " << historic.back().second;
}

void LocalMap::ComputeGlobalPositionOfVoxels()
{
    cv::Mat reference_kf_position = reference_kf()->GetCameraCenter();

    float half_map = (map_lenght_/2)*voxel_size_;
    float half_voxel = voxel_size_/2;

    float map_center_x = reference_kf_position.at<float>(0);
    float map_center_y = reference_kf_position.at<float>(1);
    float map_center_z = reference_kf_position.at<float>(2);

    unique_lock<mutex> lock(map_mutex);
    for(int x=0;x<map_lenght_;x++){
        for(int y=0;y<map_lenght_;y++){
            for(int z=0;z<map_lenght_;z++){
                vector<float> position_global;
                map_[x][y][z].position_global(x*voxel_size_-half_map+map_center_x+half_voxel, y*voxel_size_-half_map+map_center_y+half_voxel, z*voxel_size_-half_map+map_center_z+half_voxel);
            }
        }
    }
}

void LocalMap::SetVoxelAsFree(vector<int> position)
{
    if(IsInsideMap(position))
    {
        MapVoxel* voxel = this->GetMapVoxel(position);
        voxel->TurnToFree();
        voxels_free_.insert(voxel);
    }
}

void LocalMap::Reset()
{
    reference_kf_ = nullptr;
    this->ClearMap();
}

void LocalMap::ClearMap()
{
    unique_lock<mutex> lock(map_mutex);
    for(int x=0;x<map_lenght_;x++)
        for(int y=0;y<map_lenght_;y++)
            for(int z=0;z<map_lenght_;z++)
                map_[x][y][z].Clear();


    voxels_obstacle_.clear();
    voxels_free_.clear();

    uav_local_voxel_ = nullptr;
    goal_local_voxel_ = nullptr;

    this->UpdateLocalMapCode();
}

float LocalMap::voxel_size()
{
    return voxel_size_;
}

MapVoxel* LocalMap::GetMapVoxel(int x, int y, int z)
{
    if(!IsInsideMap(x, y, z)) return nullptr;
    return &map_[x][y][z];
}

MapVoxel* LocalMap::GetMapVoxel(vector<int> p)
{
    if(!IsInsideMap(p)) return nullptr;
    return &map_[p[0]][p[1]][p[2]];
}

void LocalMap::SetUavPoseMat(cv::Mat value)
{
    unique_lock<mutex> lock(map_mutex);
    uav_global_pose_mat_ = value;

    cv::Mat rwc;
    rwc = value.rowRange(0,3).colRange(0,3).t();

    vector<float> q = ORB_SLAM2::Converter::toQuaternion(rwc);

    float e1 = 2*q[1]*q[3] - 2*q[2]*q[0];
    float e2 = 1 - 2*pow(q[2], 2) - 2*pow(q[1], 2);

    uav_yaw_ = atan2(e1,e2);
}

float LocalMap::uav_yaw()
{
    unique_lock<mutex> lock(map_mutex);
    return uav_yaw_;
}

double LocalMap::get_kf_yaw(cv::Mat value)
{
    cv::Mat rwc;
    rwc = value.rowRange(0,3).colRange(0,3).t();

    vector<float> q = ORB_SLAM2::Converter::toQuaternion(rwc);

    float e1 = 2*q[1]*q[3] - 2*q[2]*q[0];
    float e2 = 1 - 2*pow(q[2], 2) - 2*pow(q[1], 2);

    return  atan2(e1,e2);
}

MapVoxel* LocalMap::goal_local_voxel()
{
    return goal_local_voxel_;
}

void LocalMap::EraseGoal()
{
    goal_local_voxel_ = nullptr;
}

bool LocalMap::IsInsideMap(int x, int y, int z)
{
    if(x >= 0 && x < map_lenght_ && y >= 0 && y < map_lenght_ && z >= 0 && z < map_lenght_) return true;
    return false;
}

bool LocalMap::IsInsideMap(vector<int> p)
{
    if(p[0] >= 0 && p[0] < map_lenght_ && p[1] >= 0 && p[1] < map_lenght_ && p[2] >= 0 && p[2] < map_lenght_) return true;
    return false;
}

void LocalMap::UpdateLocalMapCode()
{
    local_map_code_++;
}

void LocalMap::UpdateWriteCode()
{
    write_code_++;
}

ORB_SLAM2::KeyFrame *LocalMap::reference_kf()
{
    return reference_kf_;
}

void LocalMap::reference_kf(ORB_SLAM2::KeyFrame *reference_kf)
{
    reference_kf_ = reference_kf;
    this->ComputeGlobalPositionOfVoxels();
}

int LocalMap::pad_half_map()
{
    return pad_half_map_;
}

float LocalMap::grid_scale()
{
    return grid_scale_;
}

float LocalMap::real_scale()
{
    return real_scale_;
}

float LocalMap::EuclidianDistance(float p1x, float p1y, float p1z, float p2x, float p2y, float p2z)
{
    return float(sqrt(pow(p1x-p2x, 2)+pow(p1y-p2y, 2)+pow(p1z-p2z, 2)));
}

float LocalMap::EuclidianDistance(vector<float> p1, vector<float> p2)
{
    return float(sqrt(pow(p1[0]-p2[0], 2)+pow(p1[1]-p2[1], 2)+pow(p1[2]-p2[2], 2)));
}

float LocalMap::EuclidianDistance(vector<int> p1, vector<int> p2)
{
    return float(sqrt(pow(p1[0]-p2[0], 2)+pow(p1[1]-p2[1], 2)+pow(p1[2]-p2[2], 2)));
}

int LocalMap::map_lenght()
{
    return map_lenght_;
}

pair<MapVoxel, bool> LocalMap::uav_local_voxel()
{
    unique_lock<mutex> lock(map_mutex);
    if(uav_local_voxel_ == nullptr)
        return std::make_pair(*(new MapVoxel), false);
    return std::make_pair(*uav_local_voxel_, true);
}

std::set<MapVoxel *> LocalMap::voxels_obstacle()
{
    unique_lock<mutex> lock(map_mutex);
    return voxels_obstacle_;
}

std::set<MapVoxel *> LocalMap::voxels_free()
{
    unique_lock<mutex> lock(map_mutex);
    return voxels_free_;
}

/*##############################
 * GET CURRENT OPENGL CAMERA MATRIX
 */
void LocalMap::GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M)
{
    unique_lock<mutex> lock(map_mutex);
    if(!uav_global_pose_mat_.empty())
    {
        cv::Mat rwc(3,3,CV_32F);
        cv::Mat twc(3,1,CV_32F);

        rwc = uav_global_pose_mat_.rowRange(0,3).colRange(0,3).t();
        twc = -rwc*uav_global_pose_mat_.rowRange(0,3).col(3);

        M.m[0] = rwc.at<float>(0,0);
        M.m[1] = rwc.at<float>(1,0);
        M.m[2] = rwc.at<float>(2,0);
        M.m[3]  = 0.0;

        M.m[4] = rwc.at<float>(0,1);
        M.m[5] = rwc.at<float>(1,1);
        M.m[6] = rwc.at<float>(2,1);
        M.m[7]  = 0.0;

        M.m[8] = rwc.at<float>(0,2);
        M.m[9] = rwc.at<float>(1,2);
        M.m[10] = rwc.at<float>(2,2);
        M.m[11]  = 0.0;

        M.m[12] = twc.at<float>(0);
        M.m[13] = twc.at<float>(1);
        M.m[14] = twc.at<float>(2);
        M.m[15]  = 1.0;
    }
    else
        M.SetIdentity();
}