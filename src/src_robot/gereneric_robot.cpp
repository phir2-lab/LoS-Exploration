#include "src_robot/gereneric_robot.h"

GenericRobot::GenericRobot(){
    ready_ = false;
    slam_ready_ = false;
}

void GenericRobot::slam_ready(bool slam_ready)
{
    slam_ready_ = slam_ready;
}
