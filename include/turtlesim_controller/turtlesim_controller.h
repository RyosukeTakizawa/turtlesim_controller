#ifndef _TURTLESIM_CONTROLLER_H_
#define _TURTLESIM_CONTROLLER_H_

#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>

class TurtlesimController
{
public:
    TurtlesimController();
    void process();

private:
    // method
    void pose_callback(const turtlesim::PoseConstPtr&);
    void record_reset();
    geometry_msgs::Twist draw_circle();
    geometry_msgs::Twist draw_square();

    // parameter
    ros::NodeHandle private_nh;
    double hz;
    std::string mode;
    double value_x;
    double value_z;

    double square_length;

    // member
    ros::NodeHandle nh;
    ros::Publisher cmd_vel_pub;
    ros::Subscriber pose_sub;
    turtlesim::Pose current_pose;
    double record_distance;
    double record_orientation;
};

#endif // _TURTLESIM_CONTROLLER_H_
