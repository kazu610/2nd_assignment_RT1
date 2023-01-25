#include "ros/ros.h"
#include <unistd.h>
#include "nav_msgs/Odometry.h"
#include "assignment_2_2022/Info.h"

//Variables for the corrent position and velocity of the robot
double x, y, vel_x, vel_y, freq;

void data_Callback(const nav_msgs::Odometry::ConstPtr& data){

	//substitute the info from topic /odom
	x = data->pose.pose.position.x;
	y = data->pose.pose.position.y;
	vel_x = data->twist.twist.linear.x;
	vel_y = data->twist.twist.linear.y;
	
}

int main(int argc, char **argv){
	
	//Initialize the publisher node(a2)
	ros::init(argc, argv, "publisher");
	ros::NodeHandle n;

	//Subscribe from /odom and publish robot_data
	ros::Subscriber sub = n.subscribe("/odom", 1, data_Callback);
	ros::Publisher pub = n.advertise<assignment_2_2022::Info>("robot_data",1);
	
	//rate for publisher
	ros::param::get("freq", freq);
	ros::Rate rate(freq);

	while(ros::ok()){
	
		//publish the robot info(position and velocity)
		assignment_2_2022::Info data;
		data.x = x;
		data.y = y;
		data.vel_x = vel_x;
		data.vel_y = vel_y;
		pub.publish(data);
		
		rate.sleep(); //Sleep function
		ros::spinOnce();
	}
return 0;
}
