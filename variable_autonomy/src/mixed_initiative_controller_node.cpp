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


class MixedInitiativeController
{
public:
    MixedInitiativeController() ;

private:

    void loaCallback(const std_msgs::Int8::ConstPtr& msg);
    void robotVelOptimalCallback(const geometry_msgs::Twist::ConstPtr& msg);
    void robotVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
    void computeCostCallback(const ros::TimerEvent&);

    int loa_;
    bool valid_loa_;
    std_msgs::Bool loa_change_;
    std_msgs::Float64 vel_error_msg_;

    ros::NodeHandle n_;
    ros::Subscriber loa_sub_ ,vel_robot_sub_ , vel_robot_optimal_sub_;
    ros::Publisher loa_pub_, loa_change_pub_, vel_error_pub_;
    ros::Timer compute_cost_;

    geometry_msgs::Twist cmdvel_robot_, cmdvel_for_robot_, cmdvel_optimal_;
    actionlib_msgs::GoalID cancelGoal_;

};

MixedInitiativeController::MixedInitiativeController()
{

    loa_change_.data = false;
    valid_loa_ = false;

    loa_change_pub_ = n_.advertise<std_msgs::Bool>("/loa_change", 1);
    vel_error_pub_ = n_.advertise<std_msgs::Float64>("/vel_error", 1);

    loa_sub_ = n_.subscribe("/control_mode", 5, &MixedInitiativeController::loaCallback, this); // the current LOA
    vel_robot_sub_ = n_.subscribe("/cmd_vel", 5 , &MixedInitiativeController::robotVelCallback, this); // current velocity of the robot.
    vel_robot_optimal_sub_ = n_.subscribe("/cmd_vel_optimal", 5 , &MixedInitiativeController::robotVelOptimalCallback, this); // The optimal velocity e.g. perfect move_base

    // The ros Duration controls the period in sec. that the cost will compute. currently 10hz
    compute_cost_ = n_.createTimer(ros::Duration(0.1), &MixedInitiativeController::computeCostCallback, this);

}

void MixedInitiativeController::loaCallback(const std_msgs::Int8::ConstPtr& msg)
{

    switch (msg->data)
    {
    case 0:
    {
        loa_ = 0;
        valid_loa_ = true;
        ROS_INFO("Stop robot");
        break;
    }
    case 1:
    {
        loa_ = 1;
        valid_loa_ = true;
        ROS_INFO("Control mode: Teleoperation");
        break;
    }
    case 2:
    {
        loa_ = 2;
        valid_loa_ = true;
        ROS_INFO("Control mode: Autonomy");
        break;
    }
    default:
    {
        valid_loa_ = false;
        ROS_INFO("Please choose a valid control mode.");
    }
    }
}




void MixedInitiativeController::robotVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    cmdvel_robot_ = *msg;

}

void MixedInitiativeController::robotVelOptimalCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    cmdvel_optimal_ = *msg;
}

// Where magic happens, it computes the cost to judge in switching LAO
void MixedInitiativeController::computeCostCallback(const ros::TimerEvent&)
{ 
    vel_error_msg_.data = cmdvel_optimal_.linear.x - cmdvel_robot_.linear.x;
    vel_error_msg_.data = fabs(vel_error_msg_.data);

    if (vel_error_msg_.data > 0.04)
    {
        loa_change_.data = true;
        loa_change_pub_.publish(loa_change_);
    }

    else {
        loa_change_.data = false;
        loa_change_pub_.publish(loa_change_);
    }
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "mixed_initiative_controller");
    MixedInitiativeController controller_obj;

    ros::Rate r(20); // 20 hz
    while (ros::ok())
    {
        ros::spinOnce();
        r.sleep();
    }

}

