/*!
 BLABLABLA
 */


#include "ros/ros.h"
#include "std_msgs/String.h"

#include <cmath>
#include <iostream>
#include <algorithm>

#include <geometry_msgs/Twist.h>
#include "std_msgs/Bool.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Float64.h"
#include <actionlib_msgs/GoalID.h>


class Controller
{
public:
    Controller() ;

private:

    void modeCallback(const std_msgs::Int8& msg);
    void teleopCallback(const geometry_msgs::Twist& msg);
    void navCallback(const geometry_msgs::Twist::ConstPtr& msg);
    void robotVelOptimalCallback(const geometry_msgs::Twist::ConstPtr& msg);
    void robotVelCallback(const geometry_msgs::Twist& msg);

    int control_mode_;
    bool valid_mode_;
    std_msgs::Bool loa_change_;
    std_msgs::Float64 vel_error_msg_;

    ros::NodeHandle n_;
    ros::Subscriber control_mode_sub_, vel_teleop_sub_, vel_nav_sub_ ,vel_robot_sub_ , vel_robot_optimal_sub_;
    ros::Publisher vel_for_robot_pub_ , cancelGoal_pub_, explorationCancel_pub_, loa_change_pub_;

    geometry_msgs::Twist cmdvel_robot_, cmdvel_for_robot_, cmdvel_optimal_;
    actionlib_msgs::GoalID cancelGoal_;

};

Controller::Controller()
{
    valid_mode_ = true;
    loa_change_.data = false;
    control_mode_ = 0 ; //  stop/idle mode.

    vel_for_robot_pub_ = n_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    cancelGoal_pub_ = n_.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 5);
    loa_change_pub_ = n_.advertise<std_msgs::Bool>("/loa_change", 1);
    //vel_error_pub_ = n_.advertise<std_msgs::Float64>("/vel_error", 1);
    //explorationCancel_pub_ = n_.advertise<actionlib_msgs::GoalID>("/explore_server/cancel", 5);

    /* Subscribes to:
   * "/control_mode" to take the LOA (from joystick)
   * "/teleop/cmd_vel" to take the velocity coming from the teleoperation
   * "/navigation/cmd_vel" to take the velocity coming out of a navigation controller
  */
    control_mode_sub_ = n_.subscribe("/control_mode", 5, &Controller::modeCallback,this);
    vel_teleop_sub_ = n_.subscribe("/teleop/cmd_vel", 5, &Controller::teleopCallback,this);
    vel_nav_sub_ = n_.subscribe("/navigation/cmd_vel",5, &Controller::navCallback,this);
    vel_robot_sub_ = n_.subscribe("/cmd_vel", 5 , &Controller::robotVelCallback, this);
    vel_robot_optimal_sub_ = n_.subscribe("/cmd_vel_optimal", 5 , &Controller::robotVelOptimalCallback, this);
}

void Controller::modeCallback(const std_msgs::Int8& msg)
{

    switch (msg.data)
    {
    case 0:
    {
        control_mode_ = 0;
        valid_mode_ = true;
        ROS_INFO("Stop robot");
        break;
    }
    case 1:
    {
        control_mode_ = 1;
        valid_mode_ = true;
        ROS_INFO("Control mode: Teleoperation");
        break;
    }
    case 2:
    {
        control_mode_ = 2;
        valid_mode_ = true;
        ROS_INFO("Control mode: Autonomy");
        break;
    }
    default:
    {
        valid_mode_ = false;
        ROS_INFO("Please choose a valid control mode.");
    }
    }
}

void Controller::navCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    //cmdvel_optimal_ = *msg ;
    //vel_optimal_pub_.publish(cmdvel_optimal_);

    if (control_mode_ == 2)
    {
        cmdvel_for_robot_.linear.x = msg->linear.x;
        cmdvel_for_robot_.angular.z = msg->angular.z;
        vel_for_robot_pub_.publish(cmdvel_for_robot_);
    }
    if (control_mode_ == 0)
    {
        cmdvel_for_robot_.linear.x = 0;
        cmdvel_for_robot_.angular.z = 0;
        vel_for_robot_pub_.publish(cmdvel_for_robot_);
        cancelGoal_pub_.publish(cancelGoal_);
        explorationCancel_pub_.publish(cancelGoal_);
    }
}

void Controller::teleopCallback(const geometry_msgs::Twist& msg)
{
    if (control_mode_ == 1)
    {
        cmdvel_for_robot_.linear.x = msg.linear.x;
        cmdvel_for_robot_.angular.z = msg.angular.z;
        vel_for_robot_pub_.publish(cmdvel_for_robot_);
    }
    else if (control_mode_ == 0)
    {
        cmdvel_for_robot_.linear.x = 0;
        cmdvel_for_robot_.angular.z = 0;
        vel_for_robot_pub_.publish(cmdvel_for_robot_);
        cancelGoal_pub_.publish(cancelGoal_);
        explorationCancel_pub_.publish(cancelGoal_);
    }
    else
    {
        cmdvel_for_robot_.linear.x = 0;
        cmdvel_for_robot_.angular.z = 0;
        vel_for_robot_pub_.publish(cmdvel_for_robot_);
    }
}

void Controller::robotVelCallback(const geometry_msgs::Twist& msg)
{
    cmdvel_robot_ = msg;

    double error;
    error = cmdvel_optimal_.linear.x - cmdvel_robot_.linear.x;
    error = fabs(error);

    if (error>0.04)
    {
        loa_change_.data=true;
        loa_change_pub_.publish(loa_change_);
    }

    else {
        loa_change_.data=false;
        loa_change_pub_.publish(loa_change_);
    }
}

void Controller::robotVelOptimalCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    cmdvel_optimal_ = *msg;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "mixed_initiative_controller");
    Controller controller_obj;

    ros::Rate r(10); // 20 hz
    while (ros::ok())
    {
        ros::spinOnce();
        r.sleep();
    }

}

