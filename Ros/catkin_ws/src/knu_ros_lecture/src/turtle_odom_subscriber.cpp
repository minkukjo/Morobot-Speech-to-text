#include <ros/ros.h>
#include <turtlesim/Pose.h>
#include <iomanip>
#include <nav_msgs/Odometry.h>
#include <boost/thread/mutex.hpp>

boost::mutex mutex;
nav_msgs::Odometry g_odom;

void odomMsgCallback(const nav_msgs::Odometry &msg)
{
    // receive a '/odom' message with the mutex
    mutex.lock(); 
    {
        g_odom = msg;
	ROS_INFO_STREAM(std::setprecision(2) << std::fixed <<
		"Position x , y = (" << msg.pose.pose.position.x << "," << msg.pose.pose.position.y << ")" <<
		" Orientation Angle = " << msg.pose.pose.orientation.w );
    } 
    mutex.unlock();
}



int main(int argc, char **argv){

	ros::init(argc,argv,"odom_receive");
	ros::NodeHandle nh;
	
	ros::Subscriber sub = nh.subscribe("/odom", 1000, &odomMsgCallback);

	ros::spin();

	return 0;
}
