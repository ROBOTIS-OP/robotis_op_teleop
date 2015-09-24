//based on teleop_base_keyboard by Willow Garage, Inc.
#include <termios.h>
#include <signal.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/poll.h>

#include <boost/thread/thread.hpp>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/JointState.h>
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>


class RobotisOPTeleopXBOX360
{
public:
    RobotisOPTeleopXBOX360();

private:
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

    ros::NodeHandle nh_;

    int linear_x_, linear_y_, angular_l_, angular_r_;
    double l_scale_, a_scale_;
    ros::Publisher vel_pub_;
    ros::Subscriber joy_sub_;

};


RobotisOPTeleopXBOX360::RobotisOPTeleopXBOX360():
    linear_x_(1),
    linear_y_(0),
    angular_l_(2),
    angular_r_(5)
{

    nh_.param("axis_linear_forwards", linear_x_, linear_x_);
    nh_.param("axis_linear_sidewards", linear_y_, linear_y_);
    nh_.param("axis_angular_right", angular_r_, angular_r_);
    nh_.param("axis_angular_left", angular_l_, angular_l_);
    nh_.param("scale_angular", a_scale_, a_scale_);
    nh_.param("scale_linear", l_scale_, l_scale_);


    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("robotis_op/cmd_vel", 1);


    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &RobotisOPTeleopXBOX360::joyCallback, this);

}

void RobotisOPTeleopXBOX360::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    geometry_msgs::Twist vel;
    vel.angular.z = a_scale_*(joy->axes[angular_r_] - joy->axes[angular_l_]);
    vel.linear.x = l_scale_*joy->axes[linear_x_];
    vel.linear.y = l_scale_*joy->axes[linear_y_];
    vel_pub_.publish(vel);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "robotis_op_teleop_xbox360_node");
    RobotisOPTeleopXBOX360 robotis_op_teleop_xbox360_node;

    ros::spin();
}
