#ifndef GENERIC_ROBOT_H
#define GENERIC_ROBOT_H

#include <iostream>
#include <vector>
#include <unistd.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>

class GenericRobot
{
public:
    GenericRobot();

    virtual void Run() = 0;

    virtual void RequestTask(std::pair<int, std::vector<float>> task) = 0;

    virtual void RequestFinish() = 0;

    virtual bool ready() = 0;

    virtual cv::Mat GetImage() = 0;
    virtual std::vector<float> real_position() = 0;

    virtual bool idle() = 0;

    virtual void moving(bool moving) = 0;
    virtual bool moving() = 0;

    virtual void ChangeVel(float x_vel, float y_vel, float z_vel, float yaw) = 0;

    void slam_ready(bool slam_ready);

    enum PossibleTasks{
        WAIT=-1,
        INITIALIZE=0,
        TURN=1,
        MOVEBYVEL=2,
    };

protected:
    bool ready_;
    bool slam_ready_;

};

#endif // GENERIC_ROBOT_H
