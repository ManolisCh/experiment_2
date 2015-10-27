#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/variate_generator.hpp>
#include <geometry_msgs/PoseWithCovarianceStamped.h>





class LaserNoise
{
public:
    LaserNoise()
    {
        //randomGen_.seed(time(NULL)); // seed the generator
        laser_sub_ = n_.subscribe<sensor_msgs::LaserScan>("scan", 50, &LaserNoise::laserReadCallBAck, this);
        pose_sub_ = n_.subscribe<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose", 50, &LaserNoise::poseCallback, this);

        scan_pub_ = n_.advertise<sensor_msgs::LaserScan>("scan_with_noise", 50);

        timerNoise_ = n_.createTimer(ros::Duration(10) , &LaserNoise::timerNoiseCallback, this, true, false);

        areaTriger_ = 0, timerTriger_ =0, timerActivated_= 0 ;
    }

private:

    //  boost::mt19937 randomGen_;

    ros::NodeHandle n_;
    ros::Subscriber laser_sub_ , pose_sub_;
    ros::Publisher scan_pub_ ;
    sensor_msgs::LaserScan addedNoiseScan_;
    ros::Timer timerNoise_ ;

    void laserReadCallBAck(const sensor_msgs::LaserScan::ConstPtr& msg);
    double GaussianKernel(double mu,double sigma), uniformNoise_;
    void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
    void timerNoiseCallback(const ros::TimerEvent&);

    bool areaTriger_ , timerTriger_, timerActivated_; // uniformTriger_;
};



void LaserNoise::laserReadCallBAck(const sensor_msgs::LaserScan::ConstPtr& msg)

{

    double sigma;
    double oldRange;
    addedNoiseScan_ = *msg ;


    // Guassian noise added
    if (areaTriger_ == 1 && timerTriger_ == 0)
    {
        for (int i=0; i < addedNoiseScan_.ranges.size() ; i++)

        {
            sigma = addedNoiseScan_.ranges[i] * 0.2; // Proportional standard deviation
            oldRange = addedNoiseScan_.ranges[i] ;
            addedNoiseScan_.ranges[i] = addedNoiseScan_.ranges[i] + GaussianKernel(0,sigma);

            if (addedNoiseScan_.ranges[i] > addedNoiseScan_.range_max)
            { addedNoiseScan_.ranges[i] = addedNoiseScan_.range_max;}

            else if (addedNoiseScan_.ranges[i] < addedNoiseScan_.range_min)
            { addedNoiseScan_.ranges[i] = oldRange;}
        }
    }


    addedNoiseScan_.header.stamp = ros::Time::now();

    scan_pub_.publish(addedNoiseScan_);


}

void LaserNoise::poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)

{

    if ( (msg->pose.pose.position.x < 534.98) && (msg->pose.pose.position.x > 533.75)
         && (msg->pose.pose.position.y < 55.12) && (msg->pose.pose.position.y > 47.90) )

    {

        areaTriger_ =1;
        timerNoise_.setPeriod(ros::Duration(10));
        timerNoise_.start();


    }
    else
        areaTriger_ = 0;
        timerTriger_ = 0;
}


void LaserNoise::timerNoiseCallback(const ros::TimerEvent&)
{
    //uniformTriger_ = 1; // activate uniform noise

    // alternates between noise and no noise
    // if (noiseTriger_ == 0)
    //    noiseTriger_ = 1;
    // else
    timerTriger_ = 1;
    //noiseTriger_ = 0;
    ROS_INFO("TIMER ACTIVATED");

}

// Utility for adding noise
double LaserNoise::GaussianKernel(double mu,double sigma)
{
    // using Box-Muller transform to generate two independent standard normally disbributed normal variables

    double U = (double)rand()/(double)RAND_MAX; // normalized uniform random variable
    double V = (double)rand()/(double)RAND_MAX; // normalized uniform random variable
    double X = sqrt(-2.0 * ::log(U)) * cos( 2.0*M_PI * V);
    //double Y = sqrt(-2.0 * ::log(U)) * sin( 2.0*M_PI * V); // the other indep. normal variable
    // we'll just use X
    // scale to our mu and sigma
    X = sigma * X + mu;
    return X;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "laser_noise");
    LaserNoise lasernoise;

    ros::spin();
}
