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


class ControlDataLogger
{
public:
    ControlDataLogger() ;

private:

    void robotCmdVelOptimalCallback(const geometry_msgs::Twist::ConstPtr& msg);
    void robotCmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
    void computeCostCallback(const ros::TimerEvent&);


    double vel_error_, angular_error_ ;

    std_msgs::Float64 linear_vel_error_msg_, angular_vel_error_msg_;

    ros::NodeHandle n_;
    ros::Subscriber cmdvel_robot_sub_ , cmdvel_robot_optimal_sub_;
    ros::Publisher vel_error_pub_, angluar_error_pub_;
    ros::Timer compute_cost_;

    geometry_msgs::Twist cmdvel_robot_, cmdvel_optimal_;

};

ControlDataLogger::ControlDataLogger()
{
    cmdvel_robot_sub_ = n_.subscribe("/cmd_vel", 5 , &ControlDataLogger::robotCmdVelCallback, this); // current velocity of the robot.
    cmdvel_robot_optimal_sub_ = n_.subscribe("/cmd_vel_optimal", 5 , &ControlDataLogger::robotCmdVelOptimalCallback, this); // The optimal velocity e.g. perfect move_base

    vel_error_pub_ = n_.advertise<std_msgs::Float64>("/vel_error", 1);
    angluar_error_pub_ = n_.advertise<std_msgs::Float64>("/angular_error", 1);

    // The ros Duration controls the period in sec. that the cost, error etc will be computed. currently 10hz
    compute_cost_ = n_.createTimer(ros::Duration(0.2), &ControlDataLogger::computeCostCallback, this);
}


// logging currect cmdvel of robot
void ControlDataLogger::robotCmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    cmdvel_robot_ = *msg;
}

// logging expert/optimal cmdvel
void ControlDataLogger::robotCmdVelOptimalCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    cmdvel_optimal_ = *msg;
}

// it computes the cmd vel error
void ControlDataLogger::computeCostCallback(const ros::TimerEvent&)
{ 
    //linear
    vel_error_ = cmdvel_optimal_.linear.x - cmdvel_robot_.linear.x;
    vel_error_ = fabs(vel_error_);
    linear_vel_error_msg_.data = vel_error_;
    vel_error_pub_.publish(linear_vel_error_msg_);

    // angular
    angular_error_ = fabs(cmdvel_optimal_.angular.z) - fabs(cmdvel_robot_.angular.z) ;
    angular_error_ = fabs(angular_error_);
    angular_vel_error_msg_.data = angular_error_;
    angluar_error_pub_.publish(angular_vel_error_msg_);
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "mixed_initiative_controller");
    ControlDataLogger controller_obj;

    ros::Rate r(10); // 10 hz
    while (ros::ok())
    {
        ros::spinOnce();
        r.sleep();
    }

}

