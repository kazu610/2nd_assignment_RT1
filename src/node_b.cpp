#include "ros/ros.h"
#include <unistd.h>
#include <sstream>
#include <math.h>
#include "assignment_2_2022/Goal.h"

bool goal_number(assignment_2_2022::Goal::Request &req, assignment_2_2022::Goal::Response &res){
	int r, c;
	ros::param::get("/reached_goal", r);
	ros::param::get("/cancelled_goal", c);
	res.total_r = r + req.count_r;
	res.total_c = c + req.count_c;
	ros::param::set("/reached_goal", res.total_r);
	ros::param::set("/cancelled_goal", res.total_c);
	ROS_INFO("Reached goals:%d, Cancelled goals:%d", res.total_r, res.total_c);
	
	return true;
}

int main (int argc, char **argv){
	ros:: init(argc, argv, "goal_info");
	ros::NodeHandle n;
	ros::ServiceServer service = n.advertiseService("/goal_info", goal_number);
	ros::spin();
	
	return 0;
}
