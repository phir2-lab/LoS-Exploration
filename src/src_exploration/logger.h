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

#ifndef LOGGER_H
#define LOGGER_H

#include <chrono>
#include <vector>

#include "System.h"
#include "src_exploration/util_configurations.h"

using namespace std;

class Logger
{
public:
    Logger();

    Logger(UtilConfigurations configuration);

    double get_time();

    void slam(ORB_SLAM2::System* slam_);

    void Initilize();

    void InitializeLogExplorationStates();
    void LogExplorationStates(int state);

    void InitializeLogPlanner();
    void LogPlanner(int state);

    void InitializeLogKeyframes();
    void LogKeyframes(int total, int closed);

    void InitializeLogPoints();
    void LogPoints();

    void InitializeLogKfsLocalMap();
    void LogKfsLocalMap(int kfs);

    void InitializeLogTimeCreateLocalMap();
    void StartLogTimeCreateLocalMap();
    void LogTimeCreateLocalMap();

    void InitializeLogTimePlanning();
    void StartLogTimePlanning();
    void LogTimePlanning();

    void InitializeLogTimeLocalPath();
    void StartLogTimeLocalPath();
    void LogTimeLocalPath();

    void InitializeLogTimeGlobalPath();
    void StartLogTimeGlobalPath();
    void LogTimeGlobalPath();

private:

    bool enable_;

    std::chrono::steady_clock::time_point pass_;

    std::chrono::steady_clock::time_point time_local_map_;
    std::chrono::steady_clock::time_point time_planning_;
    std::chrono::steady_clock::time_point time_local_path_;
    std::chrono::steady_clock::time_point time_global_path_;

    ORB_SLAM2::System* slam_;

};

#endif // LOGGER_H
