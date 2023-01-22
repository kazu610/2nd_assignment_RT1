#include "ros/ros.h"
#include <unistd.h>
#include <sstream>
#include <math.h>
#include "assignment_2_2022/Info.h"

void data_Callback(const assignment_2_2022::Info& data){
	float target_x, target_y, dist, ave_vel;
	ros::param::get("des_pos_x",target_x);
	ros::param::get("des_pos_y",target_y);
	dist = sqrd(pow(data->x - target_x, 2.0)+(data->y - target_y, 2.0));
	ave_vel = sqrd(pow(data->vel_x,2.0)+(data->vel_y, 2.0)); //not sure
	ROS_INFO("Distance:%f, Velocity:%f", dist, ave_vel);
}

int main(int argc, char **argv){

	ros::init(argc, argv, "node_c");
	ros::NodeHandle n;

	ros::subscriber sub = n.subscribe("robot_data", 1, data_Callback);

	ros::spin();
	return 0;
}
