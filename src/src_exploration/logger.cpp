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

#include <src_exploration/logger.h>

Logger::Logger()
{
    enable_ = false;
}

Logger::Logger(UtilConfigurations configuration)
{
    enable_ = configuration.GetBool("save_log");

    if(!enable_) return;

    pass_ = chrono::steady_clock::now();
    this->Initilize();
}

void Logger::Initilize()
{
    this->InitializeLogExplorationStates();
    this->InitializeLogKeyframes();
    this->InitializeLogKfsLocalMap();
    this->InitializeLogPoints();
    this->InitializeLogPlanner();
    this->InitializeLogTimeCreateLocalMap();
    this->InitializeLogTimePlanning();
    this->InitializeLogTimeLocalPath();
    this->InitializeLogTimeGlobalPath();
}

void Logger::slam(ORB_SLAM2::System *slam)
{
    this->slam_ = slam;
}

double Logger::get_time()
{
    return chrono::duration_cast<std::chrono::duration<double> >(chrono::steady_clock::now() - pass_).count();
}

void Logger::InitializeLogExplorationStates()
{
    if(!enable_) return;

    ofstream f;
    f.open("logs/exploration_state.txt");
    f << "time(s);state" << endl;
    f.close();
}

void Logger::LogExplorationStates(int state)
{
    if(!enable_) return;

    ofstream f;
    f.open("logs/exploration_state.txt", fstream::app);

    f << fixed << this->get_time() << ";" << state << endl;

    f.close();
}

void Logger::InitializeLogPlanner()
{
    if(!enable_) return;

    ofstream f;
    f.open("logs/planner.txt");
    f << "time(s);planner_event" << endl;
    f.close();
}

void Logger::LogPlanner(int state)
{
    if(!enable_) return;

    ofstream f;
    f.open("logs/planner.txt", fstream::app);

    f << fixed << this->get_time() << ";" << state << endl;

    f.close();
}

void Logger::InitializeLogKeyframes()
{
    if(!enable_) return;

    ofstream f;
    f.open("logs/keyframes.txt");
    f << "time(s);total_keyframes;open_keyframes;close_keyframes" << endl;
    f.close();
}

void Logger::LogKeyframes(int total, int closed)
{
    if(!enable_) return;

    ofstream f;
    f.open("logs/keyframes.txt", fstream::app);

    f << fixed << this->get_time() << ";" << total << ";" << total-closed << ";" << closed << endl;

    f.close();
}

void Logger::InitializeLogPoints()
{
    if(!enable_) return;

    ofstream f;
    f.open("logs/points.txt");
    f << "time(s);total_points;open_points;reached_points" << endl;
    f.close();
}

void Logger::LogPoints()
{
    if(!enable_) return;

    ofstream f;
    f.open("logs/points.txt", fstream::app);

    const vector<ORB_SLAM2::MapPoint*> points = slam_->GetMap()->GetAllMapPoints();

    int size = points.size();
    int reached = 0;

    for(int i=0; i<size; i++)
    {
        if(points[i]->reached()) reached++;
    }

    f << fixed << this->get_time() << ";" << size << ";" << size-reached << ";" << reached << endl;

    f.close();
}

void Logger::InitializeLogTimeCreateLocalMap()
{
    if(!enable_) return;

    ofstream f;
    f.open("logs/time_local_map.txt");
    f << "time(s);total_time(ms);kfs_used" << endl;
    f.close();
}

void Logger::StartLogTimeCreateLocalMap()
{
    if(!enable_) return;
    time_local_map_ = chrono::steady_clock::now();
}

void Logger::LogTimeCreateLocalMap()
{
    if(!enable_) return;

    double diff = chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - time_local_map_).count();

    if(diff == 0) return;

    ofstream f;
    f.open("logs/time_local_map.txt", fstream::app);

    f << fixed << this->get_time() << ";" << diff << endl;

    f.close();
}

void Logger::InitializeLogKfsLocalMap()
{
    if(!enable_) return;

    ofstream f;
    f.open("logs/kfs_local_map.txt");
    f << "time(s);kfs" << endl;
    f.close();
}

void Logger::LogKfsLocalMap(int kfs)
{
    if(!enable_) return;

    ofstream f;
    f.open("logs/kfs_local_map.txt", fstream::app);

    f << fixed << this->get_time() << ";" << kfs << endl;

    f.close();
}

void Logger::InitializeLogTimePlanning()
{
    if(!enable_) return;

    ofstream f;
    f.open("logs/time_planning.txt");
    f << "time(s);total_time(ms)" << endl;
    f.close();
}

void Logger::StartLogTimePlanning()
{
    if(!enable_) return;
    time_planning_ = chrono::steady_clock::now();
}

void Logger::LogTimePlanning()
{
    if(!enable_) return;

    double diff = chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - time_planning_).count();

    if(diff == 0) return;

    ofstream f;
    f.open("logs/time_planning.txt", fstream::app);

    f << fixed << this->get_time() << ";" << diff << endl;

    f.close();
}


void Logger::InitializeLogTimeLocalPath()
{
    if(!enable_) return;

    ofstream f;
    f.open("logs/time_local_path.txt");
    f << "time(s);total_time(ms)" << endl;
    f.close();
}

void Logger::StartLogTimeLocalPath()
{
    if(!enable_) return;
    time_local_path_ = chrono::steady_clock::now();
}

void Logger::LogTimeLocalPath()
{
    if(!enable_) return;

    double diff = chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - time_local_path_).count();

    if(diff == 0) return;

    ofstream f;
    f.open("logs/time_local_path.txt", fstream::app);

    f << fixed << this->get_time() << ";" << diff << endl;

    f.close();
}

void Logger::InitializeLogTimeGlobalPath()
{
    if(!enable_) return;

    ofstream f;
    f.open("logs/time_global_path.txt");
    f << "time(s);total_time(ms)" << endl;
    f.close();
}

void Logger::StartLogTimeGlobalPath()
{
    if(!enable_) return;
    time_global_path_ = chrono::steady_clock::now();
}

void Logger::LogTimeGlobalPath()
{
    if(!enable_) return;

    double diff = chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - time_global_path_).count();

    if(diff == 0) return;

    ofstream f;
    f.open("logs/time_global_path.txt", fstream::app);

    f << fixed << this->get_time() << ";" << diff << endl;

    f.close();
}
