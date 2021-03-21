#include "turtlesim_controller/turtlesim_controller.h"

TurtlesimController::TurtlesimController() : private_nh("~")
{
    // parameter
    private_nh.param("hz", hz, {10});
    private_nh.param("mode", mode, {"circle"});

    // publisher
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1);

    // print parameter
    std::cout << "=== turtlesim_controller ===" << std::endl;
    std::cout << "hz: " << hz << std::endl;
    std::cout << std::endl;
}

geometry_msgs::Twist TurtlesimController::draw_circle()
{
    geometry_msgs::Twist twist;
    twist.linear.x = 1.0;
    twist.angular.z = 1.0;

    return twist;
}

void TurtlesimController::process()
{
    ros::Rate loop_rate(hz);

    while(ros::ok())
    {
        geometry_msgs::Twist twist;
        if(mode == "circle")
        {
            twist = draw_circle();
        }
        else
        {
            std::cout << "invalid mode" << std::endl;
        }

        cmd_vel_pub.publish(twist);

        std::cout << "--- publish ---" << std::endl;
        std::cout << "linear.x:  " << twist.linear.x << std::endl;
        std::cout << "angular.z: " << twist.angular.z << std::endl;

        ros::spinOnce();
        loop_rate.sleep();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "turtlesim_controller");
    TurtlesimController turtlesim_controller;
    turtlesim_controller.process();

    return 0;
}
