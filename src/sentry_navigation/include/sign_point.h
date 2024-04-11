#ifndef SIGN_POINT
#define SIGN_POINT

#include "ros/ros.h"
#include <iostream>

class SignPoint{
public:
    void sendGoal(double goal_x, double goal_y, double goal_yaw);
};

#endif // SIGN_POINT