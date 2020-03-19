/**
 * This file is part of LoS-Exploration.
 * This file is based on ORB-SLAM2/Viewer.cc <https:// github.com/raulmur/ORB_SLAM2>, see GPLv3 license below.
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

/**
 * This file is part of ORB-SLAM2.
 *
 * Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
 * For more information see <https:// github.com/raulmur/ORB_SLAM2>
 *
 * ORB-SLAM2 is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ORB-SLAM2 is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with ORB-SLAM2. If not, see <http:// www.gnu.org/licenses/>.
**/

#include <src_exploration/viewer.h>

mutex ExplorationViewer::stop_mutex;

ExplorationViewer::ExplorationViewer(UtilConfigurations configuration, LocalMap *map, int *state, PathPlanner *planner)
{
	m_point_size_ = configuration.GetFloat("viewer_mPointSize");
    m_viewpoint_x_ = configuration.GetFloat("viewer_mViewpointX");
    m_viewpoint_y_ = configuration.GetFloat("viewer_mViewpointY");
    m_viewpoint_z_ = configuration.GetFloat("viewer_mViewpointZ");
    m_viewpoint_f_ = configuration.GetFloat("viewer_mViewpointF");
    m_keyframe_size_ = configuration.GetFloat("viewer_mKeyFrameSize");
    m_keyframe_line_width_ = configuration.GetFloat("viewer_mKeyFrameLineWidth");
    
    local_map_ = map;
    exploration_state_ = state;
    this->planner_ = planner;

    slam_ = nullptr;

    stopped_ = false;
    stop_requested_ = false;

    name_count_ = 0;

    force_finish_ = false;
    viewer_online_ = true;
}

//##############################
// RUN
// Keeps interface runnnig in a loop
void ExplorationViewer::Run()
{
    pangolin::CreateWindowAndBind("Exploration",1024,768);

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // 3D Mouse handler requires depth testing to be enabled
    glEnable(GL_DEPTH_TEST);

    // Issue specific OpenGl we might need
    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // Menu itens
    pangolin::CreatePanel("menu_exploration").SetBounds(0.0,1.0,0.0,pangolin::Attach::Pix(195));
    pangolin::Var<bool> menu_kf("menu_exploration.Keyframes",true,true);
    pangolin::Var<bool> menu_points("menu_exploration.Points",true,true);
    pangolin::Var<bool> menu_all_lines_of_sight("menu_exploration.Lines of Sight (All)",false,true);
    pangolin::Var<bool> menu_kf_lines_of_sigh("menu_exploration.Lines of Sight (KF)",false,true);

    pangolin::Var<bool> menu_local_map_uav("menu_exploration.LM UAV Pos",true,true);
    pangolin::Var<bool> menu_local_map_obstacles("menu_exploration.LM Obstacles",true,true);
    pangolin::Var<bool> menu_local_map_free("menu_exploration.LM Free",false,true);
    pangolin::Var<bool> menu_local_map_path("menu_exploration.LM Path",true,true);
    pangolin::Var<bool> menu_local_map_goal("menu_exploration.LM Goal",true,true);
    pangolin::Var<bool> menu_local_map_borders("menu_exploration.LM Borders",false,true);

    pangolin::Var<bool> menu_points_style("menu_exploration.Points reached",true,true);
    pangolin::Var<bool> menu_follow("menu_exploration.Follow",false,true);
    pangolin::Var<bool> menu_save_snap("menu_exploration.Save a Snapshot",false,true);
    pangolin::Var<bool> menu_white("menu_exploration.White Version",true,true);
    pangolin::Var<bool> menu_finish("menu_exploration.Finish System",false,true);

    pangolin::CreatePanel("menu_status").SetBounds(0,pangolin::Attach::Pix(35),pangolin::Attach::Pix(195),0.0);
    pangolin::Var<std::string> menu_exploration_state("menu_status. ","-------");

    // Defines Camera Render Object (for view / scene browsing)
    pangolin::OpenGlRenderState s_cam(
                pangolin::ProjectionMatrix(1024,768,m_viewpoint_f_,m_viewpoint_f_,512,389,0.1,1000),
                pangolin::ModelViewLookAt(m_viewpoint_x_,m_viewpoint_y_,m_viewpoint_z_, 0,0,0,0.0,-1.0, 0.0)
                );

    // Adds named OpenGL viewport to window and provide 3D Handler
    pangolin::View& d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f/768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));

    pangolin::OpenGlMatrix twc;
    twc.SetIdentity();

    ORB_SLAM2::Map* orb_map = nullptr;

    bool follow_cam = true;

    while(true)
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        if(menu_white)
            glClearColor(1,1,1,1);
        else
            glClearColor(0.7f,0.7f,0.7f,1.0f);

        viewer_online_ = !menu_finish;

        switch(*exploration_state_)
        {
            case NOT_READY:
                menu_exploration_state = "Not Ready";
                break;

            case WAITING:
                menu_exploration_state = "Waiting";
                break;

            case UPDATING_LOCAL_MAP:
                menu_exploration_state = "Updating Local Map";
                break;

            case PLANNING:
                menu_exploration_state = "Planning";
                break;

            case NAVIGATING:
                menu_exploration_state = "Navigating";
                break;

            case LOOKING_AROUND:
                menu_exploration_state = "Looking Around";
                break;

            case CHECKING_FINISH:
                menu_exploration_state = "Checking if is Finished";
                break;
				
			case START_MOVEMENT:
                menu_exploration_state = "Starting a movement";
                break;

            case FINISHED:
                menu_exploration_state = "Finished";
                break;

            default:
                menu_exploration_state = "Unknow";
                break;
        }

        if(this->Stop() || this->IsStopped())
        {
            usleep(50000);
        }
        else if(!slam_)
        {
            usleep(50000);
        }
        else if(slam_->GetTrackingState() <= 1)
        {
            usleep(50000);
        }
        else
        {
            local_map_->GetCurrentOpenGLCameraMatrix(twc);

            if(menu_follow && follow_cam)
            {
                s_cam.Follow(twc);
            }
            else if(menu_follow && !follow_cam)
            {
                s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(m_viewpoint_x_,m_viewpoint_y_,m_viewpoint_z_, 0,0,0,0.0,-1.0, 0.0));
                s_cam.Follow(twc);
                follow_cam = true;
            }
            else if(!menu_follow && follow_cam)
            {
                follow_cam = false;
            }

            d_cam.Activate(s_cam);

            if(orb_map == nullptr) orb_map = slam_->GetMap();

            const vector<ORB_SLAM2::KeyFrame*> orb_kfs = orb_map->GetAllKeyFrames();

            if(!orb_kfs.empty())
            {

                // DRAWS KEYFRAMES AND THE LINES OF SIGHT
                if(menu_kf || menu_kf_lines_of_sigh || menu_all_lines_of_sight)
                {
                    ORB_SLAM2::KeyFrame* kf;
                    cv::Mat kf_position;

                    ORB_SLAM2::KeyFrame* kf_parent;
                    cv::Mat pose_parent;

                    set<ORB_SLAM2::MapPoint*> kf_points;
                    cv::Mat point_position;

                    for(size_t i=0; i<orb_kfs.size(); i++)
                    {
                        kf = orb_kfs[i];

                        if(kf->isBad()) continue;

                        kf_position = kf->GetCameraCenter();

                        if(menu_kf) // Keyframes
                        {
                            DrawKfPosition(kf);

                            // Draws parent connection
                            kf_parent = kf->GetParent();
                            if(kf_parent)
                            {
                                pose_parent = kf_parent->GetCameraCenter();
                                glColor3f(0,1,0);
                                glBegin(GL_LINES);
                                glVertex3f(pose_parent.at<float>(0), pose_parent.at<float>(1), pose_parent.at<float>(2));
                                glVertex3f(kf_position.at<float>(0), kf_position.at<float>(1), kf_position.at<float>(2));
                                glEnd();
                            }
                        }

                        if(menu_kf_lines_of_sigh || menu_all_lines_of_sight) // Lines of Sight
                        {
                            if(local_map_->reference_kf() == nullptr && !menu_all_lines_of_sight) continue;

                            if(local_map_->reference_kf() != nullptr)
                                if(local_map_->reference_kf()->mnId != kf->mnId && !menu_all_lines_of_sight) continue;

                            kf_points = kf->GetMapPoints();

                            for(set<ORB_SLAM2::MapPoint*>::iterator it = kf_points.begin(); it != kf_points.end(); it++)
                            {
                                if((*it)->isBad()) continue;
                                point_position = (*it)->GetWorldPos();
                                glColor3f(0,0,0);
                                glBegin(GL_LINES);
                                glVertex3f(point_position.at<float>(0), point_position.at<float>(1), point_position.at<float>(2));
                                glVertex3f(kf_position.at<float>(0), kf_position.at<float>(1), kf_position.at<float>(2));
                                glEnd();
                            }

                        }
                    } // end KF for
                }

                // DRAWS POINTS
                if(menu_points)
                {
                    vector<ORB_SLAM2::MapPoint*>  points = orb_map->GetAllMapPoints();

                    cv::Mat point_position;
                    for(size_t j=0; j<points.size();j++)
                    {
                        glPointSize(m_point_size_);
                        glBegin(GL_POINTS);

                        point_position = points[j]->GetWorldPos();

                        float color_factor = 0;
						if(menu_points_style) // Show the points by coloring them with reached or not
                        {
                            if(points[j]->reached()) glColor3f(0, 0, 0);
                            else glColor3f(1, 0, 0);
                        }
                        else // Show the points in white or black
                        {
                            if(menu_white)
                                glColor3f(0, 0, 0);
                            else
                                glColor3f(1, 1, 1);
                        }

                        glVertex3f(point_position.at<float>(0), point_position.at<float>(1), point_position.at<float>(2));
                        glEnd();
                    }

                }

                if(local_map_->reference_kf() != nullptr)
                {
                    // DRAWS GOAL
                    if(menu_local_map_goal)
                        DrawGoal();

                    // DRAWS FREE AREA
                    if(menu_local_map_free)
                        DrawFree();

                    // DRAWS OBSTACLES
                    if(menu_local_map_obstacles)
                        DrawObstacles();

                    // DRAWS PATH
                    if(menu_local_map_path && *exploration_state_ == NAVIGATING)
                        DrawPath(menu_local_map_uav);

                    // DRAWS UAV
                    if(menu_local_map_uav)
                        DrawUAV();

                    // DRAWS LOCAL MAP BORDERS
                    if(menu_local_map_borders)
                        DrawBorders();

                }

                // SAVES A SNAP
                if(menu_save_snap)
                {
                    std::ostringstream oss;
                    oss << "img/";
                    oss << std::setfill('0') << std::setw(4) << ++name_count_;
                    std::string s = oss.str();
                    pangolin::SaveWindowOnRender(s);

                    menu_save_snap = false;
                }
            }
        }
        pangolin::FinishFrame();
    }
}

void ExplorationViewer::DrawPath(bool menu_local_map_uav)
{
    float map_voxel_size = local_map_->voxel_size();

    MapVoxel* voxel;
    float angle;
    float look_to[2];

    vector<PathStep> local_path = planner_->local_path();

    vector<float> point;
    for(uint i=0; i<local_path.size(); i++)
    {
        voxel = local_path[i].voxel;

        pair<MapVoxel, bool> uav_pair = local_map_->uav_local_voxel();

        bool equal_to_uav = false;
        if(uav_pair.second)
        {
            if(menu_local_map_uav &&
               uav_pair.first.position_local()[0] == voxel->position_local()[0] &&
               uav_pair.first.position_local()[1] == voxel->position_local()[1] &&
               uav_pair.first.position_local()[2] == voxel->position_local()[2]) equal_to_uav = true;
        }

        float sheer = 1;
        if(local_path[i].finished)
            sheer = 0.2;

        point = voxel->position_global();

        if(!equal_to_uav)
        {
            float color[4] = {0.2, 0.2, 1, sheer};
            DrawCube(point[0], point[1], point[2], color);
        }

        angle = local_path[i].yaw*M_PI/180;
        look_to[0] = sin(angle)*map_voxel_size*3+point[0];
        look_to[1] = cos(angle)*map_voxel_size*3+point[2];

        glColor4f(0,0,0.6, sheer);
        glBegin(GL_LINES);
        glVertex3f(point[0], point[1], point[2]);
        glVertex3f(look_to[0], point[1], look_to[1]);
        glEnd();
    }
}

void ExplorationViewer::DrawUAV()
{
    float map_voxel_size = local_map_->voxel_size();

    if(local_map_->uav_local_voxel().second)
    {
        vector<float> point = local_map_->uav_local_voxel().first.position_global();

        DrawCube(point[0], point[1], point[2], color_uav_);

        float uav_yaw = local_map_->uav_yaw();

        float look_to[2];
        look_to[0] = sin(uav_yaw)*map_voxel_size*3+point[0];
        look_to[1] = cos(uav_yaw)*map_voxel_size*3+point[2];

        glColor3f(1,0,0);
        glBegin(GL_LINES);
        glVertex3f(point[0], point[1], point[2]);
        glVertex3f(look_to[0], point[1], look_to[1]);
        glEnd();
    }

}

void ExplorationViewer::DrawObstacles()
{
    std::set<MapVoxel*> voxels = local_map_->voxels_obstacle();

    vector<float> point;
    for(std::set<MapVoxel*>::iterator it = voxels.begin(); it != voxels.end(); ++it)
    {
        point = (*it)->position_global();
        DrawCube(point[0], point[1], point[2], color_obstacle_);
    }

}

void ExplorationViewer::DrawFree()
{
    std::set<MapVoxel*> voxels = local_map_->voxels_free();

    vector<float> point;
    for(std::set<MapVoxel*>::iterator it = voxels.begin(); it != voxels.end(); ++it)
    {
        float freeRate = (*it)->free_rate();
        float color[4] = {1, 1, 1*freeRate, 1};
        point = (*it)->position_global();
        DrawCube(point[0], point[1], point[2], color);
    }
}

void ExplorationViewer::DrawGoal()
{
    MapVoxel* voxel = local_map_->goal_local_voxel();
    if(voxel)
    {
        vector<float> point;
        point = local_map_->goal_local_voxel()->position_global();
        DrawCube(point[0], point[1], point[2], color_goal_);
    }

}

void ExplorationViewer::DrawCube(float x, float y, float z,  float color[4])
{
    glBegin(GL_QUADS);
        // Bottom
        DrawCubeFace(x, y, z, 0, color);

        // Top
        DrawCubeFace(x, y, z, 1, color);

        // Back
        DrawCubeFace(x, y, z, 2, color);

        // Front
        DrawCubeFace(x, y, z, 3, color);

        // Right
        DrawCubeFace(x, y, z, 4, color);

        // Left
        DrawCubeFace(x, y, z, 5, color);
    glEnd();  // End of drawing color-cube
}

void ExplorationViewer::DrawCubeFace(float x, float y, float z, int side, float color[4])
{
    float size = local_map_->voxel_size()/2;
    float factor = 0.2;

    switch(side) {
        // Bottom
        case 0:
            glColor4f(color[0]-(2*factor), color[1]-(2*factor), color[2]-(2*factor), color[3]);
            glVertex3f(x+size, y+size, z-size);
            glVertex3f(x-size, y+size, z-size);
            glVertex3f(x-size, y+size, z+size);
            glVertex3f(x+size, y+size, z+size);
        break;

        // Top
        case 1:
            glColor4f(color[0]+factor, color[1]+factor, color[2]+factor, color[3]);
            glVertex3f(x+size, y-size, z+size);
            glVertex3f(x-size, y-size, z+size);
            glVertex3f(x-size, y-size, z-size);
            glVertex3f(x+size, y-size, z-size);
        break;

        // Back
        case 2:
            glColor4f(color[0]-factor, color[1]-factor, color[2]-factor, color[3]);
            glVertex3f(x+size, y+size, z+size);
            glVertex3f(x-size, y+size, z+size);
            glVertex3f(x-size, y-size, z+size);
            glVertex3f(x+size, y-size, z+size);
        break;

        // Front
        case 3:
            glColor4f(color[0]-factor, color[1]-factor, color[2]-factor, color[3]);
            glVertex3f(x+size, y-size, z-size);
            glVertex3f(x-size, y-size, z-size);
            glVertex3f(x-size, y+size, z-size);
            glVertex3f(x+size, y+size, z-size);
        break;

        // Right
        case 4:
            glColor4f(color[0], color[1], color[2], color[3]);
            glVertex3f(x+size, y+size, z-size);
            glVertex3f(x+size, y+size, z+size);
            glVertex3f(x+size, y-size, z+size);
            glVertex3f(x+size, y-size, z-size);
        break;

        // Left
        case 5:
            glColor4f(color[0], color[1], color[2], color[3]);
            glVertex3f(x-size, y+size, z+size);
            glVertex3f(x-size, y+size, z-size);
            glVertex3f(x-size, y-size, z-size);
            glVertex3f(x-size, y-size, z+size);
        break;
    }
}

void ExplorationViewer::DrawBorders()
{
    int map_lenght = local_map_->map_lenght();
    float map_voxel_size = local_map_->voxel_size();
    float half_map = (map_lenght/2.0)*map_voxel_size;

    cv::Mat reference_kf_position = local_map_->reference_kf()->GetCameraCenter();

    float map_center_x = reference_kf_position.at<float>(0);
    float map_center_y = reference_kf_position.at<float>(1);
    float map_center_z = reference_kf_position.at<float>(2);

    float q1[3];
    q1[0] = 100*map_voxel_size-half_map+map_center_x;
    q1[1] = 50*map_voxel_size-half_map+map_center_y;
    q1[2] = 100*map_voxel_size-half_map+map_center_z;

    float q2[3];
    q2[0] = map_voxel_size-half_map+map_center_x;
    q2[1] = 50*map_voxel_size-half_map+map_center_y;
    q2[2] = 100*map_voxel_size-half_map+map_center_z;

    float q3[3];
    q3[0] = map_voxel_size-half_map+map_center_x;
    q3[1] = 50*map_voxel_size-half_map+map_center_y;
    q3[2] = map_voxel_size-half_map+map_center_z;

    float q4[3];
    q4[0] = 100*map_voxel_size-half_map+map_center_x;
    q4[1] = 50*map_voxel_size-half_map+map_center_y;
    q4[2] = map_voxel_size-half_map+map_center_z;

    glColor3f(0,0,0);
    glBegin(GL_LINES);
    glVertex3f(q1[0], q1[1], q1[2]);
    glVertex3f(q2[0], q2[1], q2[2]);

    glVertex3f(q2[0], q2[1], q2[2]);
    glVertex3f(q3[0], q3[1], q3[2]);

    glVertex3f(q3[0], q3[1], q3[2]);
    glVertex3f(q4[0], q4[1], q4[2]);

    glVertex3f(q1[0], q1[1], q1[2]);
    glVertex3f(q4[0], q4[1], q4[2]);
    glEnd();
}

void ExplorationViewer::DrawKfPosition(ORB_SLAM2::KeyFrame* kf)
{
    const float &w = m_keyframe_size_;
    const float h = w*0.75;
    const float z = w*0.6;

    cv::Mat twc = kf->GetPoseInverse().t();

    glPushMatrix();

    glMultMatrixf(twc.ptr<GLfloat>(0));

    glLineWidth(m_keyframe_line_width_);

    if(kf->closed) glColor3f(0.5f,0.5f,0.5f);
    else glColor3f(0.0f,0.0f,1.0f);

    glBegin(GL_LINES);
    glVertex3f(0,0,0);
    glVertex3f(w,h,z);
    glVertex3f(0,0,0);
    glVertex3f(w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,h,z);

    glVertex3f(w,h,z);
    glVertex3f(w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(-w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(w,h,z);

    glVertex3f(-w,-h,z);
    glVertex3f(w,-h,z);
    glEnd();

    glPopMatrix();
}

bool ExplorationViewer::force_finish()
{
    return force_finish_;
}

void ExplorationViewer::slam(ORB_SLAM2::System *slam)
{
    this->slam_ = slam;
}

bool ExplorationViewer::viewer_online()
{
    return viewer_online_;
}

void ExplorationViewer::AdjustUsingScale(float scale)
{
    m_keyframe_size_ *= scale*3;
}

void ExplorationViewer::RequestStop()
{
    unique_lock<mutex> lock(stop_mutex);
    stop_requested_ = true;
}

void ExplorationViewer::ReleaseStop(){
    unique_lock<mutex> lock(stop_mutex);
    stopped_ = false;
}

bool ExplorationViewer::Stop()
{
    unique_lock<mutex> lock(stop_mutex);
    if(stop_requested_ == true){
        stop_requested_ = false;
        stopped_ = true;
        return true;
    }
    return false;
}

bool ExplorationViewer::IsStopped()
{
    unique_lock<mutex> lock(stop_mutex);
    return stopped_;
}
