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

#include <fl/Headers.h>

class ControlDataLogger
{
public:
    ControlDataLogger() ;

private:

    void loaCallback(const std_msgs::Int8::ConstPtr& msg);
    void robotVelOptimalCallback(const geometry_msgs::Twist::ConstPtr& msg);
    void robotVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
    void computeCostCallback(const ros::TimerEvent&);

    int loa_, number_timesteps_ , count_timesteps_;
    bool valid_loa_;
    double error_sum_, error_average_, a_,vel_error_ , vel_error_threshold_;
    std_msgs::Bool loa_change_;
    std_msgs::Float64 error_average_msg_;

    ros::NodeHandle n_;
    ros::Subscriber loa_sub_ ,vel_robot_sub_ , vel_robot_optimal_sub_;
    ros::Publisher loa_pub_, loa_change_pub_, vel_error_pub_;
    ros::Timer compute_cost_;

    geometry_msgs::Twist cmdvel_robot_, cmdvel_for_robot_, cmdvel_optimal_;
    actionlib_msgs::GoalID cancelGoal_;

};

ControlDataLogger::ControlDataLogger()
{

    loa_change_.data = false;
    valid_loa_ = false;
    a_ = 0.08; // smoothing factor between [0,1]
    vel_error_threshold_ = 0.1;

    number_timesteps_ = 25; // # of time steps used to initialize average
    count_timesteps_ = 1; // counts the # of time steps used to initialize average


    loa_change_pub_ = n_.advertise<std_msgs::Bool>("/loa_change", 1);
    vel_error_pub_ = n_.advertise<std_msgs::Float64>("/vel_error", 1);

    loa_sub_ = n_.subscribe("/control_mode", 5, &ControlDataLogger::loaCallback, this); // the current LOA
    vel_robot_sub_ = n_.subscribe("/cmd_vel", 5 , &ControlDataLogger::robotVelCallback, this); // current velocity of the robot.
    vel_robot_optimal_sub_ = n_.subscribe("/cmd_vel_optimal", 5 , &ControlDataLogger::robotVelOptimalCallback, this); // The optimal velocity e.g. perfect move_base

    // The ros Duration controls the period in sec. that the cost will compute. currently 10hz
    compute_cost_ = n_.createTimer(ros::Duration(0.2), &ControlDataLogger::computeCostCallback, this);

}

void ControlDataLogger::loaCallback(const std_msgs::Int8::ConstPtr& msg)
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




void ControlDataLogger::robotVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    cmdvel_robot_ = *msg;

}

void ControlDataLogger::robotVelOptimalCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    cmdvel_optimal_ = *msg;
}

// Where magic happens, it computes the cost, judging to switch LAO
void ControlDataLogger::computeCostCallback(const ros::TimerEvent&)
{ 
    vel_error_ = cmdvel_optimal_.linear.x - cmdvel_robot_.linear.x;
    vel_error_ = fabs(vel_error_);

    // calculates the average used to initialize exponential moving average
    if (count_timesteps_ <= number_timesteps_)
    {
        error_sum_ += vel_error_;
        error_average_ = error_sum_ / number_timesteps_;
        count_timesteps_++;
    }
    // calculates  exponential moving average
    else if (count_timesteps_ > number_timesteps_)
    {
        error_average_ = a_ * vel_error_ + (1-a_) * error_average_;

        if ( (error_average_ > vel_error_threshold_) && (loa_change_.data == false) )
        {
            loa_change_.data = true;
            loa_change_pub_.publish(loa_change_);
            count_timesteps_ = 1; // enables re-initializaion of moving average by reseting count
            loa_change_.data = false; // resets loa_change flag
            error_sum_ = 0; // resets sumation of errors for initial estimate
            ros::Duration(10).sleep();
        }
        else if ((error_average_ < vel_error_threshold_) && loa_change_.data == true)
        {
            loa_change_.data = false;
            loa_change_pub_.publish(loa_change_);
        }

    }

    error_average_msg_.data = error_average_;
    vel_error_pub_.publish(error_average_msg_);

}


int main(int argc, char *argv[])
{

    // Fuzzylite stuff

    fl::Engine* engine = new fl::Engine;
    engine->setName("controller");

    fl::InputVariable* inputVariable = new fl::InputVariable;
    inputVariable->setEnabled(true);
    inputVariable->setName("error");
    inputVariable->setRange(0.000, 0.100);
    inputVariable->addTerm(new fl::Trapezoid("small", -1.000, 0.000, 0.040, 0.060));
    inputVariable->addTerm(new fl::Trapezoid("medium", 0.040, 0.070, 0.080, 0.090));
    inputVariable->addTerm(new fl::Trapezoid("large", 0.080, 0.090, 0.100, 0.200));
    engine->addInputVariable(inputVariable);

    fl::OutputVariable* outputVariable = new fl::OutputVariable;
    outputVariable->setEnabled(true);
    outputVariable->setName("change_LOA");
    outputVariable->setRange(-1.000, 1.000);
    outputVariable->fuzzyOutput()->setAccumulation(new fl::Maximum);
    outputVariable->setDefuzzifier(new fl::LargestOfMaximum(200));
    outputVariable->setDefaultValue(fl::nan);
    // outputVariable->setLockValidOutput(false);
    //outputVariable->setLockOutputRange(true);
    outputVariable->addTerm(new fl::Triangle("change", 0.000, 1.000, 1.000));
    outputVariable->addTerm(new fl::Triangle("no_change", -1.000, -1.000, 0.000));
    engine->addOutputVariable(outputVariable);

    fl::RuleBlock* ruleBlock = new fl::RuleBlock;
    ruleBlock->setEnabled(true);
    ruleBlock->setName("");
    ruleBlock->setConjunction(new fl::Minimum);
    ruleBlock->setDisjunction(new fl::Maximum);
    ruleBlock->setActivation(new fl::Minimum);
    ruleBlock->addRule(fl::Rule::parse("if error is small then change_LOA is no_change", engine));
    ruleBlock->addRule(fl::Rule::parse("if error is medium then change_LOA is no_change", engine));
    ruleBlock->addRule(fl::Rule::parse("if error is large then change_LOA is change", engine));
    engine->addRuleBlock(ruleBlock);


    engine->setInputValue("error", 0);
    engine->process();

    FL_LOG("Power.output = " << fl::Op::str(engine->getOutputValue("change_LOA") ) );

    //-------------------------------------------------------------------///
    ros::init(argc, argv, "mixed_initiative_controller");
    ControlDataLogger controller_obj;

    ros::Rate r(10); // 10 hz
    while (ros::ok())
    {
        ros::spinOnce();
        r.sleep();
    }

}

