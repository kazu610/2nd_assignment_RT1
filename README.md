2nd_assignment_RT1
================================

This Repository is for the second assigmnent of Research Track 1.

Visit the documentation for this repository:
https://kazu610.github.io/2nd_assignment_RT1/index.html

Introduction
------------

These are the task we have to accomplish.
1. Create a node that implements an action client to allow the user to set/cancel the goal and publishes the topic `/robot_data` (position and velocity) as a custom message by subscribing the topic `/odom`. The node can be devided into two nodes.
2. Create a node that implements a service server that prints the number of reached/cancelled goals when it is called.
3. Create a node that subscribes the topic `/robot_data` and prints the distance of the robot from the goal and the average speed. Use a parameter to set the frequency of publishing information.
4. Create a launch file to start the whole simulation and to set the frequency.


Installing and Running
----------------------

### Installing ###

Firstly, please clone this repository by using the this command.

```bash
$ git clone https://github.com/kazu610/2nd_assignment_RT1.git
```

Secondly, you need to install xterm package to open several terminals.

```bash
$ sudo apt-get install xterm
```

Thirdly, please run the ROS master and compile the program in the work space directory.

```bash
$ roscore &
$ catkin_make
```

If the package is built successfully, you can launch the simulation.

### Running ###

To run the simulation, please launch `whole_sim.launch` and you will see the environment.

```bash
$ roslaunch assignment_2_2022 whole_sim.launch
```

In addition to the main terminal where you launched, you will find 4 new windows below.

1. Gazebo: 3D simulator for ROS

![gazebo](https://user-images.githubusercontent.com/114085558/214564797-c4289448-b997-4028-b260-fe54f85acb47.png)

2. Rviz: tool for ROS visualization

![rviz](https://user-images.githubusercontent.com/114085558/214564805-d94743c1-58e0-4359-bb2c-f0362eaf1842.png)

3. Interface (node_a1) : you can set/cancel the goal

![node_a1](https://user-images.githubusercontent.com/114085558/214564800-15458386-1b5b-4612-9d23-52b75771b42d.png)

4. robot_info (node_c) : you can check the distance and average speed of the robot

![node_c](https://user-images.githubusercontent.com/114085558/214564802-9ed5cce7-4f84-4039-a6e5-d854c7eb5994.png)


Description of the implementation
---------

### Overall ###

To accomplish the task, I made 4 nodes and 1 launch file.
| Node name | Role |
|:-------------------:|:----------------:|
| Interface (node_a1) | send `goal` and `cancel` based on the user's input |
| Publisher (node_a2) | publish the topic `/robot_data` |
| Goal_info (node_b) | display the number of reached/cancelled goals when called |
| Robot_info (node_c) | publish the distance and average speed |

You can check the relationship between nodes in rqt-graph.
![rosgraph](https://user-images.githubusercontent.com/114085558/214564803-c0c7d630-d6b3-41a3-a376-e9cdc2e551fb.png)

### Interface (node_a1) ###

This node works as the action client and the service client. After the action client confirms that the action server `/reaching_goal` started, it enters the while loop to allow the user to input.  

```cpp
//Service client corresponding to the service server in goal_info node(b)
ros::ServiceClient client = n.serviceClient<assignment_2_2022::Goal>("/goal_info");
assignment_2_2022::Goal goal; //Declare

//Actionlib
actionlib::SimpleActionClient<assignment_2_2022::PlanningAction> ac("/reaching_goal", true);
ROS_INFO("Waiting for action server to start");
ac.waitForServer(); //waiting for action server start
ROS_INFO("Confirmed the server started");
assignment_2_2022::PlanningGoal target; //Declare
```

In the while loop, the user always has two choices, `'g'` and `'c'`. If the user inputs `'g'`, the user can set new goal position (x, y) and the robot will start going the new goal as soon as the action client sends the goal. On the other hand, if the user types `'c'`, the current goal will be cancelled and the service client sends the request to the server. The while loop lasts as long as `ros::ok()` is true.

```cpp
//Interface to set new goal or to cancel current goal
while(ros::ok()){
    
    //Display message and the user can choose 'g' or 'c'
    std::cout << "Please type 'g' or 'c'\n'g' setting new goal, 'c' cancelling  the current goal: " << std::endl;
    std::cin >> input;
    
    //Set new goal
    if (input == "g"){
        
        //Set x and y position
        std::cout << "Please enter new goal position (x, y): " << std::endl;
        std::cin >> x_new >> y_new;
        
        //Send new goal to the server
        target.target_pose.pose.position.x = x_new;
        target.target_pose.pose.position.y = y_new;
        ac.sendGoal(target);
        ROS_INFO("You set new goal");
        
        //Update rosparam
        ros::param::set("des_pos_x", x_new);
        ros::param::set("des_pos_y", y_new);
        
    }
    else if (input == "c"){

        //Cancel the current goal
        ac.cancelGoal();
        ROS_INFO("The goal was cancelled");

        //Service client
        goal.request.count_c = 1; //One goal was cancelled
        client.call(goal);
        
    }
    else{
        
        //Invalid input
        std::cout << "Please type valid keys, 'g' or 'c': " << std::endl;
    
    }
ros::spinOnce();
}
```

### Publisher (node_a2) ###

This node works as the publisher. I made a custom messages `Info` composed of the poaition `x`, `y`, and the velocity `vel_x`, `vel_y`. The name of the topic is `/robot_data`.

```cpp
float64 x
float64 y
float64 vel_x
float64 vel_y
```

The node needs to subscribe the topic `/odom` published by gazebo node which the topic `/robot_data` relies on. Since the frequency of publishing of `node_c` depends on the publisher, the node gets ROS parameter `freq` set in the launch file and sets the rate of publishing.

```cpp
//Subscribe from /odom and publish robot_data
ros::Subscriber sub = n.subscribe("/odom", 1, data_Callback);
ros::Publisher pub = n.advertise<assignment_2_2022::Info>("robot_data",1);

//rate for publisher
ros::param::get("freq", freq);
ros::Rate rate(freq);
```

The data subscribed from `/odom` are published as `robot_data` in the while loop. 

```cpp
void data_Callback(const nav_msgs::Odometry::ConstPtr& data){

	//substitute the info from topic /odom
	x = data->pose.pose.position.x;
	y = data->pose.pose.position.y;
	vel_x = data->twist.twist.linear.x;
	vel_y = data->twist.twist.linear.y;
	
}
```

```cpp
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
```
### Flowchart of node_a1 and node_a2 ###

![flowchart_node_a](https://user-images.githubusercontent.com/114085558/214564794-71ad3da5-84ed-42a7-959c-e56ae071494a.png)

### Goal_info (node_b) ###

This node works as the service server which publishes the number of reached/cancelled goals. I made a custom service definded as `Goal` to recieve the notification of cancelling as request and publish the goal's number as response.

```cpp
int8 count_c
---
int8 total_r
int8 total_c
```

To count the number of reached goal, the subscriber is also implemented. The topic `/reaching_goal/result` includes the status of the goal.

```cpp
//Servic server that sends the number of reached/cancelled goals 
ros::ServiceServer service = n.advertiseService("/goal_info", goal_number);

//Subscribe status from action server
ros::Subscriber sub = n.subscribe("/reaching_goal/result", 1, status_Callback);
```

Every time the node subscribes `/reaching_goal/result`, it checks if the status is `SUCCEEDED(3)`, which means the goal was reached. And it displays the goal's number and the variable `reached_goals` is incremented by one when the current goal was reached.

```cpp
void status_Callback(const assignment_2_2022::PlanningActionResult::ConstPtr& msg){
	
	//When the status is SUCCEEDED(3)
	if(msg->status.status == 3){
	
		ROS_INFO("The robot has reached the goal");
		reached_goals += 1; //One goal was reached
		ROS_INFO("Reached goals:%d, Cancelled goals:%d", reached_goals, cancelled_goals);
	
	}	
}
```

When the service client (node_a2) sends request, the variable `cancelled_goals` is incremented by one and server responds.

```cpp
bool goal_number(assignment_2_2022::Goal::Request &req, assignment_2_2022::Goal::Response &res){
	
	cancelled_goals += 1; //One goal was cancelled
	res.total_r = reached_goals;
	res.total_c =cancelled_goals;
	ROS_INFO("Reached goals:%d, Cancelled goals:%d", res.total_r, res.total_c);
	
	return true;
}
```

### Robot_info (node_c) ###

This node works as the subscriber which receives the topic `/robot_data` published by node_a2.

```cpp
//Subscribe robot position and velocity from publisher node(a2)
ros::Subscriber sub = n.subscribe("robot_data", 10, data_Callback);
```

Based on the current posotion and velocity of the robot, the distance of the robot from the goal and the average speed.

```cpp
void data_Callback(const assignment_2_2022::Info::ConstPtr& data){

	p_x = data->x;
	p_y = data->y;
	v_x = data->vel_x;
	v_y = data->vel_y;
	
	//Display current robot information
	ROS_INFO("Distance to goal: %4.2f", distance(p_x, p_y));
	ROS_INFO("Average speed: %4.2f", average_speed(v_x, v_y));
	
	count += 1;
}
```

The function `distance` computes the distance to the goal. Also the function `average_speed` computes the average speed. The variable `vel` is the sum of the speed from the beginning and `count` is the number of subscribing. The average speed `ave_vel` is computed by deviding `vel` by `count`.

```cpp
//Compute current distance between the robot and the goal
double distance(double p_x, double p_y){

	double t_x, t_y, dist;
	
	//Get goal position from ROS parameter
	ros::param::get("des_pos_x",t_x);
	ros::param::get("des_pos_y",t_y);
	
	//Calculate distance
	dist = sqrt(pow((t_x - p_x), 2.0) + pow((t_y - p_y), 2.0));
	
	return dist;
}

//Compute average speed of the robot
double average_speed(double v_x, double v_y){

	double ave_vel;
	
	//Calculate average speed
	vel += sqrt(pow(v_x,2.0) + pow(v_y, 2.0));
	ave_vel = vel/count;
	
	return ave_vel;
}
```
### Launch file ###

I created the launch file `whole_sim.launch` to launch 4 nodes and set the ROS parameter `freq` for node_c's publishing as well as existing `assignment1.launch`. The command `launch-prefix="xterm -e"` allows the node to start in new terminal.

```xml
<?xml version="1.0"?>
<launch>
    <include file="$(find assignment_2_2022)/launch/assignment1.launch" />
    
    <!-- Setting frequency for node_c(robot_info) and goal counters -->
    <param name="freq" type="double" value="1.0" />
    
    <!-- Launch nodes -->
    <node pkg="assignment_2_2022" type="node_a1" name="node_a1" output="screen" launch-prefix="xterm -e" />
    <node pkg="assignment_2_2022" type="node_a2" name="node_a2" />
    <node pkg="assignment_2_2022" type="node_b" name="node_b" output="screen" />
    <node pkg="assignment_2_2022" type="node_c" name="node_c" output="screen" launch-prefix="xterm -e" />
    

</launch>
```

Movie (x2.5)
----------------

https://user-images.githubusercontent.com/114085558/214564783-f5e21baa-bc0d-45e7-8f7c-e06098d7cc9e.mp4

Future work
-----------

Using class is one of the most biggest improvement that I can do. The project using ROS can be larger, more complicated, and edited by several members. Class allows us to sort the code out and makes code more flexible. I will study C++ more, in particular class.

