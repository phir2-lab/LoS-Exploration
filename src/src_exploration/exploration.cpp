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

#include <src_exploration/exploration.h>

mutex Exploration::pose_mutex;

Exploration::Exploration(UtilConfigurations configuration)
{
    logger_ = new Logger(configuration);

    std::cout << "EXPLORATION: " << "Initializing...";

    min_kfs_local_map_ = configuration.GetInt("min_kfs_local_map");
    min_angle_turn_around_ = configuration.GetInt("min_angle_turn_around");
    min_percent_unvisited_ = configuration.GetFloat("min_percent_unvisited");

    max_yaw_vel_ = configuration.GetFloat("uav_max_yaw_vel");
    max_vel_ = configuration.GetFloat("uav_max_vel");
    min_vel_ = configuration.GetFloat("uav_min_vel");
    uav_distance_to_voxel_ = configuration.GetFloat("uav_distance_to_voxel");

    uav_std_vel_ = configuration.GetFloat("std_vel");

    std::cout << " Local map...";
    local_map_ = new LocalMap(configuration);

    std::cout << " Path planner...";
    path_planner_ = new PathPlanner(configuration, local_map_);

    show_window_ = configuration.GetBool("exploration_show");
    if(show_window_)
    {
        std::cout << " Viewer...";
        viewer_ = new ExplorationViewer(configuration, local_map_, &exploration_state_, path_planner_);
        thread_viewer_ = new thread(&ExplorationViewer::Run, viewer_);
    }
    else std::cout << " Without viewer...";

    global_pose_changed_ = false;
    exploration_state_ = Exploration::NOT_READY;
    past_exploration_state_ = -999;

    initial_yaw_ = make_pair(0, false);

    global_navigation_ = false;

    initialized_ = false;
    finished_ = false;

    std::cout << " Done." << std::endl;
}

void Exploration::Reset()
{
    unique_lock<mutex> lock(pose_mutex);
    global_pose_changed_ = false;
    exploration_state_ = Exploration::NOT_READY;
    global_navigation_ = false;
    initialized_ = false;
    initial_yaw_ = make_pair(0, false);
    decisions_historic_.clear();
}

//##############################
// RUN
// Maintains the exploration states running
void Exploration::Run()
{
    while(exploration_state_ != Exploration::FINISHED)
    {

        // Backdoor to force the exploration to end
        if(show_window_)
        {
            if(viewer_->force_finish())
            {
                exploration_state_ = Exploration::FINISHED;
                continue;
            }
        }

        // If the slam state isn't OK
        if(slam_->GetTrackingState() != 2)
        {
            // If slam is waiting to reset, then resets the exploration viewer 
            if(slam_->GetTracker()->isWaitingToReset()){
                if(!robot_->ready()) StopRobot();
                std::cout << "EXPLORATION: " << "Reset... "  << std::endl;
                if(show_window_){
                    viewer_->RequestStop();
                    while(!viewer_->IsStopped()) usleep(5000);
                }

                local_map_->Reset();
                this->Reset();

                if(show_window_) viewer_->ReleaseStop();

                slam_->GetTracker()->releaseToReset();

                std::cout << "EXPLORATION: Done" << std::endl;
            }
            // If slam just lost, changes the exploration state to WAITING
            else
            {
                if(robot_->ready()) StopRobot();
                exploration_state_ = Exploration::WAITING;
            }
        }

        // If needs to do the initializer movement
        if(!initialized_)
        {
            std::cout <<  "EXPLORATION: " << "Need initialization" << std::endl;
            if(slam_->GetTrackingState() != 2)
                InitializerMovement(false);
            else
            {
                InitializerMovement(true);
                initialized_ = true;
            }
            continue;
        }

        if(exploration_state_ != Exploration::LOOKING_AROUND && initial_yaw_.second)
        {
            initial_yaw_ = make_pair(0, false);
        }

        // If the slam's map changed aggressively (loop closure)
		// Changes the exploration state to READY TO UPDATE
        if(slam_->MapChanged()){
            logger_->LogExplorationStates(-9999);
            StopRobot();
            exploration_state_ = Exploration::UPDATING_LOCAL_MAP;
        }

        switch (exploration_state_)
        {
            case Exploration::WAITING:
                if(slam_->GetTrackingState() == 2 && robot_->ready())
                    exploration_state_ = Exploration::UPDATING_LOCAL_MAP;
            break;

            case Exploration::UPDATING_LOCAL_MAP:
                if(global_pose_changed_)
                {
                    logger_->StartLogTimeCreateLocalMap(); // LOG INFO
                    this->UpdateLocalMap();
                    logger_->LogTimeCreateLocalMap(); // LOG INFO
                }
            break;

            case Exploration::PLANNING:
                if(global_pose_changed_)
                {
                    this->UpdateUavGlobalPoseInLocal();
                    logger_->StartLogTimePlanning(); // LOG INFO
                    this->Planning();
                    logger_->LogTimePlanning(); // LOG INFO
                }
            break;

            case Exploration::NAVIGATING:
                if(global_pose_changed_)
                {
                    // Changes the exploration station to UPDATING_LOCAL_MAP
					// if the UAV is outside the local map
                    if(!this->UpdateUavGlobalPoseInLocal())
                    {
                        std::cout << "EXPLORATION: " << "Robot is outside of the local map" << std::endl;
                        StopRobot();
                        exploration_state_ = Exploration::UPDATING_LOCAL_MAP;
                        break;
                    }
                }
                this->Navigate();
            break;

            case Exploration::LOOKING_AROUND:
                if(global_pose_changed_)
                {
					// If the UAV is outside the local map
                    // Changes the exploration station to UPDATING_LOCAL_MAP
                    if(!this->UpdateUavGlobalPoseInLocal())
                    {
                        std::cout << "EXPLORATION: " << "Robot is outside of the local map" << std::endl;
                        StopRobot();
                        exploration_state_ = Exploration::UPDATING_LOCAL_MAP;
                        break;
                    }
                }
                this->TurningAround();
            break;

        }

        if(exploration_state_ != past_exploration_state_)
        {
            logger_->LogExplorationStates(exploration_state_);
            past_exploration_state_ = exploration_state_;
        }

        usleep(1000);
    }

    finished_ = true;

    robot_->RequestFinish();
}

//##############################
// PLANNING
// Make a plan
void Exploration::Planning()
{
    bool do_global = false;
    global_navigation_ = false;
	
	// Sets to visited the unvisited closer points
    local_map_->CheckReachedPoints(); 

	// If it is the first iteration
    if(this->get_last_decision() == -999) 
    {
        local_map_->EraseGoal();
        std::cout << "PLANNER: " << "Initializes looking around" << std::endl;
        logger_->LogPlanner(2); // LOG INFO
        exploration_state_ = Exploration::LOOKING_AROUND;
        this->StoreDecision(exploration_state_);
        return;
    }

    // Tries to define a new goal and set to visited
	// the points inside saturated voxels
    bool foundGoal = local_map_->DefineNextGoal();

	// Updates the keyframes state (open or close)
	// and check if exploration is completed
    if(this->CheckFinished())
    {
        local_map_->EraseGoal();
        std::cout << "PLANNER: " << "All KF closed. Finished" << std::endl;
        exploration_state_ = Exploration::FINISHED;
        return;
    }

	// If founds a local goal, tries to define a path to it
    if(foundGoal) 
    {
        logger_->StartLogTimeLocalPath(); // LOG INFO
        bool foundPath = path_planner_->FindLocalPath(nullptr);
        logger_->LogTimeLocalPath(); // LOG INFO

        // If found a path, goes to NAVIGATING
        if(foundPath)
        {
            logger_->LogPlanner(1); // LOG INFO
            std::cout << "PLANNER: " << "Start to navigate" << std::endl;
            exploration_state_ = Exploration::NAVIGATING;
        }
        // If not found a path and the last decision taken
		// was not LOOKING AROUND, then, changes the state to LOOKING AROUND
        else if(this->get_last_decision() != Exploration::LOOKING_AROUND)
        {
            logger_->LogPlanner(2); // LOG INFO
            local_map_->EraseGoal();
            std::cout << "PLANNER: " << "Local path not founded. Looking around" << std::endl;
            exploration_state_ = Exploration::LOOKING_AROUND;
        }
        // If not found a path and the last decision taken
		// was LOOKIN AROUND, then, goes to a global planning
        else
        {
            std::cout << "PLANNER: " << "Local path not founded. Try to find a global goal and path" << std::endl;
            do_global = true;
        }
    }
    // If not found a local goal and the last decision taken
	// was not LOOKING AROUND, then, changes the state to LOOKING AROUND
    else if(this->get_last_decision() != Exploration::LOOKING_AROUND)
    {
        logger_->LogPlanner(2); // LOG INFO
        std::cout << "PLANNER: " << "Local goal not founded. Looking around" << std::endl;
        exploration_state_ = Exploration::LOOKING_AROUND;
    }
	// If does not found a local goal and the last decision taken
	// was LOOKING AROUND, goes to global planning
    else
    {
        std::cout << "PLANNER: " << "Local goal not founded. Try to find a global goal and path" << std::endl;
        do_global = true;
    }

	// If needs to find a global goal
    if(do_global)
	{ 
        ORB_SLAM2::KeyFrame* reference_kf = local_map_->reference_kf();
        cv::Mat reference_position = reference_kf->GetCameraCenter();

        const std::vector<ORB_SLAM2::KeyFrame*> all_kfs = slam_->GetMap()->GetAllKeyFrames();

        pair<ORB_SLAM2::KeyFrame*, double> goal;

        bool global_goal_found = false;

        // Takes the nearest open keyframe to be the goal
        for(uint i=0; i<all_kfs.size(); i++)
        {
            if(!all_kfs[i]->closed && !all_kfs[i]->isBad())
            {
                double distance = path_planner_->CalculateCost(reference_position, all_kfs[i]->GetCameraCenter());
                if(!global_goal_found)
                {
                    global_goal_found = true;
                    goal = make_pair(all_kfs[i], distance);
                }
                else if(distance < goal.second)
                {
                    goal = make_pair(all_kfs[i], distance);
                }
            }
        }

        // If found a keyframe to be the global goal
        if(global_goal_found)
        {
            logger_->StartLogTimeGlobalPath(); // LOG INFO

            float grid_scale = local_map_->grid_scale();
            float pad_half_map = local_map_->pad_half_map();

            // Finds a path to global goal over covisibility graph
            std::vector<ORB_SLAM2::KeyFrame*> global_path_;
            global_path_ = path_planner_->FindGlobalPath(reference_kf, goal.first);

            ORB_SLAM2::KeyFrame* goal_candidate;
            std::vector<int> goal_local_position;

            // Find a possible voxel goal (a free voxel inside local map)
            do
            {
                goal_candidate = global_path_[0];
                goal_local_position = local_map_->ConvertFromGlobalToLocal(goal_candidate->GetCameraCenter(), reference_position, grid_scale, pad_half_map);

                global_path_.erase(global_path_.begin());

                if(local_map_->IsInsideMap(goal_local_position))
                    if(local_map_->GetMapVoxel(goal_local_position)->IsFree()) break;

            } while(global_path_.size() > 0);

			// Set this voxel as the local goal
            local_map_->SetGoal(goal_local_position);

            logger_->LogTimeGlobalPath(); // LOG INFO

            logger_->StartLogTimeLocalPath(); // LOG INFO
			// Tries to find a path to the goal
            bool foundPath = path_planner_->FindLocalPath(goal_candidate);
            logger_->LogTimeLocalPath(); // LOG INFO

            // If found a path, changes the exploration state to NAVIGATING
            if(foundPath)
            {
                logger_->LogPlanner(3); // LOG INFO
                global_navigation_ = true;
                std::cout << "PLANNER: " << "Start to navigate" << std::endl;
                exploration_state_ = Exploration::NAVIGATING;
            }
			// If not found a path, changes the exploration state to FINISHED
            else
            {
                logger_->LogPlanner(4); // LOG INFO
                std::cout << "PLANNER: " << "Global path not founded. Finished" << std::endl;
                exploration_state_ = Exploration::FINISHED;
            }
        }
		// If not found a global goal, changes the exploration state to FINISHED
        else
        {
            logger_->LogPlanner(5); // LOG INFO
            std::cout << "PLANNER: " << "Global goal not founded. Finished" << std::endl;
            exploration_state_ = Exploration::FINISHED;
        }
    }

    this->StoreDecision(exploration_state_);
}

//##############################
// SET POSE
// Sets the global pose and position, and flag the change
void Exploration::SetPose(cv::Mat p)
{
    if(p.rows > 0){
        unique_lock<mutex> lock(pose_mutex);
        uav_global_pose_mat_ = p;

        cv::Mat Rwc = p.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*p.rowRange(0,3).col(3);

        uav_global_position_ = twc;

        if(exploration_state_ == Exploration::NOT_READY)
			exploration_state_ = Exploration::WAITING;

        global_pose_changed_ = true;
    }
}

//##############################
// UPDATE LOCAL MAP
// Creates a new local map with slam map informations
void Exploration::UpdateLocalMap()
{
    global_pose_changed_ = false;

    const std::vector<ORB_SLAM2::KeyFrame*> slam_kfs = slam_->GetMap()->GetAllKeyFrames();

	// If has keyframes
    if(!slam_kfs.empty()) 
    {
		// Gets reference keyframe
        ORB_SLAM2::KeyFrame* reference_kf = slam_->GetTracker()->getReferenceKF(); 

		// If it is a good keyframe
        if(!reference_kf->isBad()) 
        {
            // If the new reference keyframe doesn't have a parent
			// keeps the previous one to be the reference 
            if(reference_kf->GetParent() == nullptr && local_map_->reference_kf() != nullptr)
                reference_kf = local_map_->reference_kf();

            else
            {
                local_map_->ClearMap();

				// If the system has new scale information,
				// updates it on the local map and the viewer
                if(scale_historic_.size() > 0)
                {
                    local_map_->ChangeScale(scale_historic_);
                    scale_historic_.clear();
                    uav_distance_to_voxel_ = uav_distance_to_voxel_*local_map_->real_scale();
                    uav_vel_mod_ = uav_std_vel_/local_map_->real_scale();
                    viewer_->AdjustUsingScale(local_map_->real_scale());
                }

				// Sets the new reference to local map
                local_map_->reference_kf(reference_kf); 
                kfs_to_build_map_.clear();

                set<ORB_SLAM2::KeyFrame*> kfs_to_build_map_set;

                // Creates a set of keyframe used to build a map
                set<ORB_SLAM2::KeyFrame*> connectedKF;
                while(int(kfs_to_build_map_set.size()) < min_kfs_local_map_ && reference_kf)
                {
                    kfs_to_build_map_set.insert(reference_kf);
					
                    connectedKF = reference_kf->GetConnectedKeyFrames(); // Gets the connected keyframes

                    // Puts all connected keyframes in the set
                    for(set<ORB_SLAM2::KeyFrame*>::iterator it = connectedKF.begin(); it != connectedKF.end(); it++)
                    {
                        if((*it)->isBad()) continue;
                        kfs_to_build_map_set.insert((*it));
                    }

					// Gets the best parent to continue the process until have
					// enough keyframes in the set, or have no keyframes out the set
                    reference_kf = reference_kf->GetParent(); 
                }

                // Inserts in the set all open keyframes
                std::vector<ORB_SLAM2::KeyFrame *> open_kfs = this->OpenKeyframes();
                for(size_t i=0; i<open_kfs.size(); i++)
                    kfs_to_build_map_set.insert(open_kfs[i]);

				std::vector<ORB_SLAM2::KeyFrame*> kfs_to_build_map_(kfs_to_build_map_set.begin(), kfs_to_build_map_set.end());

                logger_->LogKfsLocalMap(kfs_to_build_map_.size());

				// Builds a local map
                for(size_t i=0; i<kfs_to_build_map_.size(); i++){
                    if(!kfs_to_build_map_[i]->isBad())
                        local_map_->UpdateMapFromKf(kfs_to_build_map_[i]);
                }

                exploration_state_ = Exploration::PLANNING;
                return;
            }
        }
    }

    // If could not build the local map for any reason,
	// changes the state to UPDATING_LOCAL_MAP
    exploration_state_ = Exploration::UPDATING_LOCAL_MAP;
}

//##############################
// UPDATE LOCAL MAP DURING NAVIGATION
// Updates the voxels with the new points of the cloud
void Exploration::UpdateLocalMapDuringNavigation()
{
    const std::vector<ORB_SLAM2::KeyFrame*> slam_kfs = slam_->GetMap()->GetAllKeyFrames();

    if(!slam_kfs.empty())
    {
        ORB_SLAM2::KeyFrame* reference_kf = slam_->GetTracker()->getReferenceKF();

        if(!reference_kf->isBad())
        {
            set<ORB_SLAM2::KeyFrame*> kfs_to_update_map_set;

            // Creates a set of keyframe used to update the map
            set<ORB_SLAM2::KeyFrame*> connectedKF;
            while(int(kfs_to_update_map_set.size()) < min_kfs_local_map_ && reference_kf)
            {
                kfs_to_update_map_set.insert(reference_kf);

                connectedKF = reference_kf->GetConnectedKeyFrames();
				
                // Puts all connected keyframes in the set
                for(set<ORB_SLAM2::KeyFrame*>::iterator it = connectedKF.begin(); it != connectedKF.end(); it++)
                {
                    if((*it)->isBad()) continue;
                    kfs_to_update_map_set.insert((*it));
                }
				
				// Gets the best parent to continue the process until have
				// enough keyframes in the set, or have no keyframes out the set
                reference_kf = reference_kf->GetParent();
            }

            std::vector<ORB_SLAM2::KeyFrame*> kfs_to_update_map(kfs_to_update_map_set.begin(), kfs_to_update_map_set.end());

            local_map_->UpdateMapDuringNavigation(kfs_to_update_map);
        }
    }
}

//##############################
// UPDATE LOCAL MAP POSE
// Updates the UAV pose and position in local map
bool Exploration::UpdateUavGlobalPoseInLocal()
{
    local_map_->SetUavPoseMat(uav_global_pose_mat());
    return local_map_->SetUavPositions(uav_global_position());
}

//##############################
// NAVIGATE
// Navigates over the path to the goal and checks the points reached
void Exploration::Navigate()
{
	// NOTICE
	// Here we have a simple control idea that moves the robot using
	// velocities in the three axes and a yaw velocity too.
	// This can be changed to more sophisticated control approaches.
	
    std::vector<PathStep> local_path;

    try {
		// Updates the voxels with the new points of the cloud
        UpdateLocalMapDuringNavigation();
		
        local_map_->CheckReachedPoints();

        local_path = path_planner_->local_path();
    } catch (...) {
        std::cout << "ERROR navigate step 1" << std::endl;
    }

    // Find next step
    PathStep next;
    int path_index;
    bool has_next = false;
    try {
        for(int i=0; i < local_path.size(); i++)
        {
            if(local_path[i].finished == false)
            {
                next = local_path[i];
                has_next = true;
                path_index = i;
                break;
            }
        }
    } catch (...) {
        std::cout << "ERROR navigate step 2" << std::endl;
    }

	// If has a next step
    if(!has_next)
    {
        try {
            StopRobot();
            exploration_state_ = Exploration::UPDATING_LOCAL_MAP;
            return;
        } catch (...) {
            std::cout << "ERROR navigate step 3" << std::endl;
        }
    }

	// Checks if is close enough to the final position
    cv::Mat uav_global_pos_mat = uav_global_position();
    std::vector<float> uav_global_pos;
    uav_global_pos.push_back(uav_global_pos_mat.at<float>(0));
    uav_global_pos.push_back(uav_global_pos_mat.at<float>(1));
    uav_global_pos.push_back(uav_global_pos_mat.at<float>(2));
    try {
        if(local_map_->EuclidianDistance(next.voxel->position_global(), uav_global_pos) < uav_distance_to_voxel_)
        {
            next.finished = true;
            path_planner_->SetStepFinished(path_index);
            return;
        }
    } catch (...) {
        std::cout << "ERROR navigate step 4" << std::endl;
    }

	// Moves the robot changing the velocities
    try {
        if(robot_->moving())
        {
            float y_delta = next.voxel->position_global()[0] - uav_global_pos[0];
            float z_delta = next.voxel->position_global()[1] - uav_global_pos[1];
            float x_delta = next.voxel->position_global()[2] - uav_global_pos[2];

            x_delta *= uav_vel_mod_;
            y_delta *= uav_vel_mod_;
            z_delta *= uav_vel_mod_;

            float x_vel = (x_delta < max_vel_)? x_delta : max_vel_;
            float y_vel = (y_delta < max_vel_)? y_delta : max_vel_;
            float z_vel = (z_delta < max_vel_)? z_delta : max_vel_;

            x_vel = (x_vel > -max_vel_)? x_vel : -max_vel_;
            y_vel = (y_vel > -max_vel_)? y_vel : -max_vel_;
            z_vel = (z_vel > -max_vel_)? z_vel : -max_vel_;

            x_vel = (x_vel > 0 && x_vel < min_vel_)? min_vel_ : x_vel;
            x_vel = (x_vel < 0 && x_vel > -min_vel_)? -min_vel_ : x_vel;

            y_vel = (y_vel > 0 && y_vel < min_vel_)? min_vel_ : y_vel;
            y_vel = (y_vel < 0 && y_vel > -min_vel_)? -min_vel_ : y_vel;

            z_vel = (z_vel > 0 && z_vel < min_vel_)? min_vel_ : z_vel;
            z_vel = (z_vel < 0 && z_vel > -min_vel_)? -min_vel_ : z_vel;

            float uav_yaw = local_map_->uav_yaw()*180/M_PI;

            float dif_yaw = next.yaw - uav_yaw;

            dif_yaw += (dif_yaw > 180) ? -360 : (dif_yaw < -180) ? 360 : 0;

            if(old_yaw != dif_yaw || old_x != x_vel || old_y != y_vel || old_z != z_vel)
            {
                old_yaw = dif_yaw;

                if(dif_yaw > 0)
                    dif_yaw = (dif_yaw > max_yaw_vel_) ? max_yaw_vel_ : dif_yaw;
                else
                    dif_yaw = (dif_yaw < -max_yaw_vel_) ? -max_yaw_vel_ : dif_yaw;

                old_x = x_vel;
                old_y = y_vel;
                old_z = z_vel;
                robot_->ChangeVel(x_vel, y_vel, z_vel, dif_yaw);
            }
        }
    } catch (...) {
        std::cout << "ERROR navigate step 5" << std::endl;
    }

	// If the robot is idle, ask to move by velocities
	// and wait until it is ready to do that
    try {
        if(robot_->idle())
        {
            old_x=old_y=old_z=old_yaw=0;

            std::vector<float> coor;
            robot_->RequestTask(std::make_pair(robot_->MOVEBYVEL, coor));

            while(robot_->idle()) usleep(100);
        }
    } catch (...) {
        std::cout << "ERROR navigate step 6" << std::endl;
    }
}

//##############################
// STOP ROBOT
// Asks the robot to stop and waits until it stops.
void Exploration::StopRobot()
{
    robot_->moving(false);
    while(!robot_->idle()) usleep(100);
}

//##############################
// INITIALIZER MOVEMENT
// Do the initializer movement with the robot, getting the scale when needed
void Exploration::InitializerMovement(bool get_scale)
{
	// If needs to get the scale
    if(get_scale)
    {
        std::cout << "EXPLORATION: " << "Getting first scale" << std::endl;
        scale_historic_.clear();

        while(!robot_->idle()) usleep(10);

        std::vector<float> robot_start_position = robot_->real_position();
        cv::Mat slam_start_position_mat = uav_global_position();
        std::vector<float> slam_start_position;
        slam_start_position.push_back(slam_start_position_mat.at<float>(0));
        slam_start_position.push_back(slam_start_position_mat.at<float>(1));
        slam_start_position.push_back(slam_start_position_mat.at<float>(2));

        // backward
        std::vector<float> coor;
        coor.push_back(1);
        robot_->RequestTask(std::make_pair(robot_->INITIALIZE, coor));

        while(robot_->idle()) usleep(10);
        while(!robot_->idle()) usleep(10);

        std::vector<float> robot_end_position = robot_->real_position();
        cv::Mat slam_end_position_mat = uav_global_position();
        std::vector<float> slam_end_position;
        slam_end_position.push_back(slam_end_position_mat.at<float>(0));
        slam_end_position.push_back(slam_end_position_mat.at<float>(1));
        slam_end_position.push_back(slam_end_position_mat.at<float>(2));

        float robot_distance = LocalMap::EuclidianDistance(robot_start_position, robot_end_position);
        float slam_distance = LocalMap::EuclidianDistance(slam_start_position, slam_end_position);

        scale_historic_.push_back(make_pair(robot_distance, slam_distance));

        robot_start_position = robot_end_position;
        slam_start_position = slam_end_position;

        // foward
        coor.clear();
        coor.push_back(-1);
        robot_->RequestTask(std::make_pair(robot_->INITIALIZE, coor));

        while(robot_->idle()) usleep(10);
        while(!robot_->idle()) usleep(10);

        robot_end_position = robot_->real_position();
        slam_end_position_mat = uav_global_position();
        slam_end_position.clear();
        slam_end_position.push_back(slam_end_position_mat.at<float>(0));
        slam_end_position.push_back(slam_end_position_mat.at<float>(1));
        slam_end_position.push_back(slam_end_position_mat.at<float>(2));

        robot_distance = LocalMap::EuclidianDistance(robot_start_position, robot_end_position);
        slam_distance = LocalMap::EuclidianDistance(slam_start_position, slam_end_position);

        scale_historic_.push_back(make_pair(robot_distance, slam_distance));

        robot_start_position = robot_end_position;
        slam_start_position = slam_end_position;

        // center
        coor.clear();
        coor.push_back(0);
        robot_->RequestTask(std::make_pair(robot_->INITIALIZE, coor));

        while(robot_->idle()) usleep(10);
        while(!robot_->idle()) usleep(10);

        robot_end_position = robot_->real_position();
        slam_end_position_mat = uav_global_position();
        slam_end_position.clear();
        slam_end_position.push_back(slam_end_position_mat.at<float>(0));
        slam_end_position.push_back(slam_end_position_mat.at<float>(1));
        slam_end_position.push_back(slam_end_position_mat.at<float>(2));

        robot_distance = LocalMap::EuclidianDistance(robot_start_position, robot_end_position);
        slam_distance = LocalMap::EuclidianDistance(slam_start_position, slam_end_position);

        scale_historic_.push_back(make_pair(robot_distance, slam_distance));
    }
	// If it is not need to get the scale
    else
	{
        std::cout << "EXPLORATION: " << "Initialize movement" << std::endl;
        std::vector<float> coor;
        coor.push_back(99);
        robot_->RequestTask(std::make_pair(robot_->INITIALIZE, coor));

        while(robot_->idle()) usleep(10);
        while(!robot_->idle()) usleep(10);
    }
}

//##############################
// TURNING AROUND
// Turn the robot around and checks the points reached during the rotation
void Exploration::TurningAround()
{
	// NOTICE
	// Here we have a simple control idea that rotates the robot inside
	// a virtual box using velocities in the three axes and a yaw velocity too.
	// This can be changed to more sophisticated control approaches.
	
    UpdateLocalMapDuringNavigation();
    local_map_->CheckReachedPoints();

    if(!initial_yaw_.second) // If is the first rotation step
    {
        float uav_yaw = local_map_->uav_yaw() * 180 / float(M_PI);

        if(path_planner_->IsBetterRotateToRight(min_angle_turn_around_))
        {
            std::vector<float> coor;
            coor.push_back(1);
            robot_->RequestTask(std::make_pair(robot_->TURN,coor));
        }
        else
        {
            std::vector<float> coor;
            coor.push_back(-1);
            robot_->RequestTask(std::make_pair(robot_->TURN,coor));
        }

        initial_yaw_ = make_pair(uav_yaw, true);
    }
    else
    {
        float uav_yaw = local_map_->uav_yaw() * 180 / float(M_PI);
        float dif_yaw = uav_yaw - initial_yaw_.first;

        dif_yaw += (dif_yaw > 180) ? -360 : (dif_yaw < -180) ? 360 : 0;

        if(abs(dif_yaw) > min_angle_turn_around_){
            initial_yaw_ = make_pair(0, false);

            robot_->moving(false);

            while(!robot_->idle()) usleep(100);

            exploration_state_ = Exploration::UPDATING_LOCAL_MAP;
        }
    }
}

//##############################
// CHECK FINISHED
// Update the keyframes state (open or close) and return false if any is open
bool Exploration::CheckFinished()
{
    const std::vector<ORB_SLAM2::KeyFrame*> slam_kfs = slam_->GetMap()->GetAllKeyFrames();

    bool finished = false;

    int closed_kfs = 0;
    if(!slam_kfs.empty())
    {
        finished = true;

        for(size_t i=0; i<slam_kfs.size(); i++)
        {
            if(!slam_kfs[i]->closed && slam_kfs[i]->UnvisitedRate() <= min_percent_unvisited_)
            {
                slam_kfs[i]->closed = true;
            }
            else if(slam_kfs[i]->UnvisitedRate() > min_percent_unvisited_){
                finished = false;
                if(slam_kfs[i]->closed)
                    slam_kfs[i]->closed = false;
            }

            if(slam_kfs[i]->closed) closed_kfs++;
        }
    }

    logger_->LogKeyframes(slam_kfs.size(), closed_kfs);
    logger_->LogPoints();

    return finished;
}

//##############################
// SET SLAM
// Sets the Slam and star run the exploration states
void Exploration::slam(ORB_SLAM2::System *slam)
{
    std::cout << "EXPLORATION: " << "Setting SLAM to exploration" << std::endl;
    this->slam_ = slam;
    this->logger_->slam(slam);

    thread_itself_ = new thread(&Exploration::Run, this);
    if(show_window_) viewer_->slam(slam);
}

//##############################
// OPEN KEYFRAMES
// Return all open keyframes
std::vector<ORB_SLAM2::KeyFrame *> Exploration::OpenKeyframes()
{
    std::vector<ORB_SLAM2::KeyFrame *> open_list;
    const std::vector<ORB_SLAM2::KeyFrame*> slam_kfs = slam_->GetMap()->GetAllKeyFrames();

    if(!slam_kfs.empty())
        for(size_t i=0; i<slam_kfs.size(); i++)
        {
            if(slam_kfs[i]->isBad()) continue;
            if(!slam_kfs[i]->closed) open_list.push_back(slam_kfs[i]);
        }

    return open_list;
}

void Exploration::StoreDecision(int d)
{
    if(decisions_historic_.size() > 3)
        decisions_historic_.erase(decisions_historic_.begin());

    decisions_historic_.push_back(d);
}

int Exploration::get_last_decision()
{
    if(decisions_historic_.size() == 0) return -999;

    return decisions_historic_.back();
}

void Exploration::robot(GenericRobot *value)
{
    robot_ = value;
}

bool Exploration::finished()
{
    return finished_;
}


cv::Mat Exploration::uav_global_position()
{
    unique_lock<mutex> lock(pose_mutex);
    return uav_global_position_;
}

cv::Mat Exploration::uav_global_pose_mat()
{
    unique_lock<mutex> lock(pose_mutex);
    return uav_global_pose_mat_;
}

bool Exploration::viewer_online()
{
    return viewer_->viewer_online();
}
