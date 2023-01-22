#include "ros/ros.h"
#include <unistd.h>
#include <sstream>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include "nav_msgs/Odometry.h"
#include "assignment_2_2022/Info.h"
#include <assignment_2_2022/PlanningAction.h>

float x, y, vel_x, vel_y;

void data_Callback(const nav_msgs::Odometry::ConstPtr& data){
	x = data->pose.pose.position.x;
	y = data->pose.pose.position.y;
	vel_x = data->twist.twist.linear.x;
	vel_y = data->twist.teist.linear.y;
}

int main(int argc, char **argv){

	ros::init(argc, argv, "node_a");
	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe("/odom", 1, data_Callback);
	ros::Publisher pub = n.advertise<assignment_2_2022::Info>("robot_data",1);
	actionlib::SimpleActionClient<assignment_2_2022::PlanningAction> ac("/reaching_goal", true);
	ac.waitForServer(); //waiting for action server start
	assignment::PlanningGoal target;
	target.pose.position.x = ;
	target.pose.position.y = ;
	ac.sendGoal(target);
	
ros::Rate loop_rate(10);

int count = 0;

while(ros::ok()){
	assignment_2_2022::Info data;
	data.x = x;
	data.y = y;
	data.vel_x = vel_x;
	data.vel_y = vel_y;
	
	pub.publish(data);
	ros::spinOnce();
	loop_rate.sleep();
	++count;
}
return 0;
}
