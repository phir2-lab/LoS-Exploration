/**
 * This file is part of LoS-Exploration.
 * This file is based on ORB-SLAM2/Viewer.h <https://github.com/raulmur/ORB_SLAM2>, see GPLv3 license below.
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

/**
 * This file is part of ORB-SLAM2.
 *
 * Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
 * For more information see <https://github.com/raulmur/ORB_SLAM2>
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
 * along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
**/

#ifndef EXPLORATIONVIEWER_H
#define EXPLORATIONVIEWER_H

#include <iostream>
#include <cmath>
#include <string>
#include <sstream>
#include <iomanip>

#include <pangolin/pangolin.h>
#include <pangolin/display/device/display_glut.h>
#include <GL/glut.h>

#include "System.h"
#include "src_exploration/local_map.h"
#include "src_exploration/path_planner.h"
#include "src_exploration/util_configurations.h"

using namespace std;

class ExplorationViewer
{
public:
    ExplorationViewer(UtilConfigurations configuration, LocalMap* map, int* state, PathPlanner *planner_);

    void Run();

    void DrawKfPosition(ORB_SLAM2::KeyFrame* kf);

    void DrawPath(bool menu_local_map_uav);

    void DrawUAV();

    void DrawObstacles();

    void DrawFree();

    void DrawBorders();

    void DrawGoal();

    void DrawCube(float x, float y, float z, float color[]);
    void DrawCubeFace(float x, float y, float z, int side, float color[]);

    void SaveImage();

    bool force_finish();
    void RequestStop();
    bool IsStopped();
    void ReleaseStop();
    bool Stop();

    void slam(ORB_SLAM2::System *slam_);

    bool viewer_online();

    void AdjustUsingScale(float scale);

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

    static std::mutex stop_mutex;

private:
    int* exploration_state_;

    LocalMap* local_map_;

    PathPlanner* planner_;

    ORB_SLAM2::System* slam_;

    int name_count_;

    bool stopped_;
    bool stop_requested_;
    bool force_finish_;

    bool viewer_online_;

    //Colors
    float color_uav_[4] = {0, 0, 0, 1};
    float color_goal_[4] = {0, 1, 0, 1};
    float color_kf_[4] = {0, 0, 1, 1};
    float color_obstacle_[4] = {1, 0, 0, 1};
    float color_path_start_[4] = {0, 0, 0.8, 1};
    float color_path_[4] = {0.4, 0.4, 0.8, 1};
    float color_obstacle_transparency_[4] = {1, 0, 0, 0.1};

    //Interface parameters
    float m_viewpoint_x_, m_viewpoint_y_, m_viewpoint_z_, m_viewpoint_f_;
    float m_keyframe_size_;
    float m_keyframe_line_width_;
    float m_point_size_;
};

#endif // EXPLORATIONVIEWER_H
