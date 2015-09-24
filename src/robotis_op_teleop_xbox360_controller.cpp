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
    void jointStatesCb(const sensor_msgs::JointState& msg);

    ros::NodeHandle nh_;

    int axis_linear_x_, axis_linear_y_, axis_angular_l_, axis_angular_r_, axis_head_pan, axis_head_tilt;
    double l_scale_, a_scale_, panh_scale_, tilt_scale_;
    ros::Publisher vel_pub_;
    ros::Subscriber joy_sub_;


    ros::Publisher tilt_pub_;
    ros::Publisher panh_pub_;
    ros::Publisher actionh_pub_;
    ros::Publisher enable_walking_pub_;

    ros::Subscriber joint_states_sub_;

    bool walking_enabled;
    float pan, tilt;


};


RobotisOPTeleopXBOX360::RobotisOPTeleopXBOX360():
    axis_linear_x_(1),
    axis_linear_y_(0),
    axis_angular_l_(2),
    axis_angular_r_(5)
{

    nh_.param("axis_linear_forwards", axis_linear_x_, axis_linear_x_);
    nh_.param("axis_linear_sidewards", axis_linear_y_, axis_linear_y_);
    nh_.param("axis_angular_right", axis_angular_r_, axis_angular_r_);
    nh_.param("axis_angular_left", axis_angular_l_, axis_angular_l_);
    nh_.param("axis_head_tilt", axis_head_tilt, axis_head_tilt);
    nh_.param("axis_head_pan", axis_head_pan, axis_head_pan);
    nh_.param("scale_angular", a_scale_, a_scale_);
    nh_.param("scale_linear", l_scale_, l_scale_);
    nh_.param("scale_pan", panh_scale_, panh_scale_);
    nh_.param("scale_tilt", tilt_scale_, tilt_scale_);
    walking_enabled = false;
    pan = 0.0;
    tilt = 0.0;

    joint_states_sub_ = nh_.subscribe("/robotis_op/joint_states", 100, &RobotisOPTeleopXBOX360::jointStatesCb, this);

    tilt_pub_ = nh_.advertise<std_msgs::Float64>("/robotis_op/j_tilt_position_controller/command", 100);
    panh_pub_ = nh_.advertise<std_msgs::Float64>("/robotis_op/j_pan_position_controller/command", 100);
    actionh_pub_ = nh_.advertise<std_msgs::Int32>("/robotis_op/start_action", 100);
    enable_walking_pub_ = nh_.advertise<std_msgs::Bool>("/robotis_op/enable_walking", 100);
    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("robotis_op/cmd_vel", 1);
    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &RobotisOPTeleopXBOX360::joyCallback, this);
}

void RobotisOPTeleopXBOX360::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    geometry_msgs::Twist vel;
    vel.angular.z = a_scale_*(joy->axes[axis_angular_r_] - joy->axes[axis_angular_l_]);
    vel.linear.x = l_scale_*joy->axes[axis_linear_x_];
    vel.linear.y = l_scale_*joy->axes[axis_linear_y_];
    vel_pub_.publish(vel);

    std_msgs::Float64 angle_msg;
    angle_msg.data = tilt+tilt_scale_*joy->axes[axis_head_tilt];
    tilt_pub_.publish(angle_msg);
    angle_msg.data = pan+panh_scale_*joy->axes[axis_head_pan];
    panh_pub_.publish(angle_msg);
}


void RobotisOPTeleopXBOX360::jointStatesCb(const sensor_msgs::JointState& msg)
{
    pan = msg.position.at(10);
    tilt = msg.position.at(21);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "robotis_op_teleop_xbox360_node");
    RobotisOPTeleopXBOX360 robotis_op_teleop_xbox360_node;

    ros::spin();
}
