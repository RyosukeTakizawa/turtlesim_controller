#ifndef _TURTLESIM_CONTROLLER_H_
#define _TURTLESIM_CONTROLLER_H_

#include <ros/ros.h>

#include <geometry_msgs/Twist.h>


class TurtlesimController
{
public:
    TurtlesimController();
    void process();

private:
    // method
    geometry_msgs::Twist draw_circle();

    // parameter
    ros::NodeHandle private_nh;
    double hz;
    std::string mode;

    // member
    ros::NodeHandle nh;
    ros::Publisher cmd_vel_pub;
};

#endif // _TURTLESIM_CONTROLLER_H_
