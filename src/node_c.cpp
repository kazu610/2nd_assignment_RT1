#include "ros/ros.h"
#include <unistd.h>
#include <sstream>
#include <math.h>
#include "assignment_2_2022/Info.h"

float p_x, p_y, v_x, v_y;
float vel = 0;
int count = 1;

void data_Callback(const assignment_2_2022::Info::ConstPtr& data){
	p_x = data->x;
	p_y = data->y;
	v_x = data->vel_x;
	v_y = data->vel_y;
	robot_status();
	
	count += 1;
}

void robot_status(){
	ROS_INFO("[Position] x=%f, y=%f", p_x, p_y);
	ROS_INFO("[Velocity] x=%f, y=%f", v_x, v_y);
	ROS_INFO("[Distance to goal] %f", distance(p_x, p_y));
	ROS_INFO("[Average speed] %f", average_speed(v_x, v_y));
}

float distance(p_x, p_y){
	ros::param::get("des_pos_x",t_x);
	ros::param::get("des_pos_y",t_y);
	dist = sqrd(pow(t_x - p_x, 2.0)+(t_y - p_y, 2.0));
	
	return dist;
}

float average_speed(v_x, v_y){
	vel += sqrd(pow(v_x,2.0)+(v_y, 2.0));
	ave_vel = vel/count;
	
	return ave_vel;
	
}

int main(int argc, char **argv){

	ros::init(argc, argv, "node_c");
	ros::NodeHandle n;

	ros::subscriber sub = n.subscribe("robot_data", 10, data_Callback);
	
	ros::param::get("freq", freq);
	ros::Rate rate(freq);
	
	ros::spin();
	return 0;
}
