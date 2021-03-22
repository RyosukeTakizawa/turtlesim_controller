#include "turtlesim_controller/turtlesim_controller.h"

TurtlesimController::TurtlesimController() : private_nh("~")
{
    // parameter
    private_nh.param("hz", hz, {10});
    private_nh.param("controller_time", controller_time, {60});
    private_nh.param("mode", mode, {"circle"});
    private_nh.param("value_x", value_x, {1.0});
    private_nh.param("value_z", value_z, {1.0});
    private_nh.param("square_length", square_length, {1.0});
    private_nh.param("triangle_x1", triangle_x1, {1.0});
    private_nh.param("triangle_y1", triangle_y1, {1.0});
    private_nh.param("triangle_x2", triangle_x2, {10.0});
    private_nh.param("triangle_y2", triangle_y2, {1.0});
    private_nh.param("triangle_x3", triangle_x3, {1.0});
    private_nh.param("triangle_y3", triangle_y3, {10.0});

    //initialize
    record_distance = 0;
    record_orientation = 0;

    // subscriber
    pose_sub = nh.subscribe("/turtle1/pose", 1, &TurtlesimController::pose_callback, this);

    // publisher
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1);

    // print parameter
    std::cout << "=== turtlesim_controller ===" << std::endl;
    std::cout << "hz: " << hz << std::endl;
    std::cout << "controller_time: " << controller_time << std::endl;
    std::cout << "mode: " << mode << std::endl;
    std::cout << "value_x: " << value_x << std::endl;
    std::cout << "value_z: " << value_z << std::endl;

    if (mode == "square")
    {
        std::cout << "square_length: " << square_length << std::endl;
    }
    else if(mode == "triangle")
    {
        std::cout << "triangle1: ( " << triangle_x1 << ", " << triangle_y1 << ")" << std::endl;
        std::cout << "triangle2: ( " << triangle_x2 << ", " << triangle_y2 << ")" << std::endl;
        std::cout << "triangle3: ( " << triangle_x3 << ", " << triangle_y3 << ")" << std::endl;
    }

    std::cout << std::endl;
}

void TurtlesimController::pose_callback(const turtlesim::PoseConstPtr &msg)
{
    current_pose = *msg;
    //std::cout << "current_pose:( " << current_pose.x << ", " << current_pose.y << ", " << current_pose.theta << ")" << std::endl;
}

void TurtlesimController::record_reset()
{
    record_distance = 0;
    record_orientation = 0;
}

geometry_msgs::Twist TurtlesimController::draw_circle()
{
    geometry_msgs::Twist twist;
    twist.linear.x = value_x;
    twist.angular.z = value_z;

    return twist;
}

geometry_msgs::Twist TurtlesimController::draw_square()
{
    if(current_pose.theta < 0)
    {
        current_pose.theta += 2.0 * M_PI;
    }
    static turtlesim::Pose previous_pose = current_pose;

    record_distance += sqrt(pow(current_pose.x - previous_pose.x, 2) + pow(current_pose.y - previous_pose.y, 2));
    record_orientation += current_pose.theta - previous_pose.theta;

    geometry_msgs::Twist twist;
    if(abs(record_distance) <= square_length)
    {
        std::cout << "[Edge] ( " << record_distance << ", " << record_orientation << ")" << std::endl;
        twist.linear.x = value_x;
        twist.angular.z = 0.0;
    }
    else if(abs(record_orientation) <= M_PI /2)
    {
        std::cout << "[Node] ( " << record_distance << ", " << record_orientation << ")" << std::endl;
        twist.linear.x = 0.0;
        twist.angular.z = value_z;
    }
    else
    {
        record_reset();
        std::cout << std::endl;
        std::cout << "===== Let's go Next node!!!!! =====" << std::endl;
    }

    previous_pose = current_pose;
    return twist;
}

geometry_msgs::Twist TurtlesimController::draw_triangle()
{
    static int point_number = 0;
    double target_x, target_y;
    switch(point_number)
    {
        case 0:
            target_x = triangle_x1;
            target_y = triangle_y1;
            break;
        case 1:
            target_x = triangle_x2;
            target_y = triangle_y2;
            break;
        case 2:
            target_x = triangle_x3;
            target_y = triangle_y3;
            break;
        default:
            target_x = 0.0;
            target_y = 0.0;
            break;
    }

    double target_theta = atan2(target_y - current_pose.y, target_x - current_pose.x);
    std::cout << "target: (" << target_x << ", " << target_y << ", " << target_theta << ")" << std::endl;
    std::cout << "current: (" << current_pose.x << ", " << current_pose.y << ", " << current_pose.theta << ")" << std::endl;

    geometry_msgs::Twist twist;

    if(abs(target_theta - current_pose.theta) > 1e-2)
    {
        twist.linear.x = 0.0;
        twist.angular.z = value_z;
    }
    else
    {
        twist.linear.x = value_x;
        twist.angular.z = 0.0;
    }

    if(sqrt(pow(target_x - current_pose.x, 2) + pow(target_y - current_pose.y, 2)) < 1e-2)
    {
        point_number++;
        if(point_number > 2)
            point_number = 0;
    }

    return twist;
}

void TurtlesimController::process()
{
    ros::Rate loop_rate(hz);
    ros::Time start = ros::Time::now();

    while(ros::ok())
    {
        ros::spinOnce();

        geometry_msgs::Twist twist;
        if(mode == "circle")
        {
            twist = draw_circle();
        }
        else if(mode == "square")
        {
            twist = draw_square();
        }
        else if(mode == "triangle")
        {
            twist = draw_triangle();
        }
        else
        {
            std::cout << "invalid mode" << std::endl;
            break;
        }

        cmd_vel_pub.publish(twist);

        //std::cout << "--- publish ---" << std::endl;
        //std::cout << "linear.x:  " << twist.linear.x << std::endl;
        //std::cout << "angular.z: " << twist.angular.z << std::endl;

        loop_rate.sleep();

        if(ros::Time::now() - start > ros::Duration(controller_time))
            break;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "turtlesim_controller");
    TurtlesimController turtlesim_controller;

    ros::Duration delay(0.5);
    delay.sleep();
    turtlesim_controller.process();

    return 0;
}
