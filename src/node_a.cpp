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

	//To publish robot_data
	ros::Subscriber sub = n.subscribe("/odom", 1, data_Callback);
	ros::Publisher pub = n.advertise<assignment_2_2022::Info>("robot_data",1);
	
	ros::ServiceClient client = n.serviceClient<assignment_2_2022::Goal>("/goal_info");
	assignment_2_2022::Goal goal; //declare
	
	double x = 1.0, y = 3.0; //goal position with the initial values 
	std::string input; //input from terminal
	actionlib::SimpleActionClient<assignment_2_2022::PlanningAction> ac("/reaching_goal", true);
	ac.waitForServer(); //waiting for action server start
	assignment::PlanningGoal target;
	target.target_pose.position.x = x;
	target.target_pose.position.y = y;
	ac.sendGoal(target);
	
	bool finished_before_timeout = ac.waitForResult(ros::Duration(40.0));
	
	ac.waitForResult(); 
	ros::Rate rate(50);

	while(!finished_before_timeout && ros::ok()){
		//publish the robot info(position and velocity)
		assignment_2_2022::Info data;
		data.x = x;
		data.y = y;
		data.vel_x = vel_x;
		data.vel_y = vel_y;
		pub.publish(data);
		
		//Actionlib part
		std::cout << "Please type 'g' or 'c'\n'g' setting new goal, 'c' cancelling  the current goal: " << std::endl;
		std::cin >> input;
		if (input == "g"){
			std::cout << "Please enter new goal position (x, y): " << std::endl;
			std::cin >> x >> y;
			//send new goal
			target.target_pose.position.x = x;
			target.target_pose.position.y = y;
			ac.snedGoal(target);
			//update rosparam
			ros::param::set("des_pos_x", x);
			ros::param::set("des_pos_y", y);
		}
		else if (input == "c"){
			ac.cancelGoal();
			ROS_INFO("The goal was cancelled");
			//service client
			goal.request.count_r = 0;
			goal.request.count_c = 1;
			client.call(goal);
		}
		else{
			std::cout << "Please type valid keys, 'g' or 'c': " << std::endl;
		}
		finished_before_timeout = ac.waitForResult(ros::Duration(40.0));
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	if (finished_before_timeout){
    		actionlib::SimpleClientGoalState state = ac.getState();
    		ROS_INFO("The robot has reached the goal: %s",state.toString().c_str());
    		//service client
    		goal.request.count_r = 1;
		goal.request.count_c = 0;
		client.call(goal);
  	}
  	else{
    		ROS_INFO("Action did not finish before the time out.");
   		ac.cancelGoal(); 
   		ROS_INFO("Goal has been cancelled");
   	}
	
return 0;
}
