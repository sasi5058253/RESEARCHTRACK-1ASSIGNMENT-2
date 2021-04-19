# RESEARCHTRACK-1ASSIGNMENT-2## Table of Contents
1. Content of Package
2. Computational Graph
3. Instruction: How to run the code
4. Robot Behaviour
5. Software Architecture
6. System's Limitation and Possible Improvement

## General Info 
This is the package to simulate the non-holonomic robot 
by using navigation stack on gazebo environment.
The robot is localized by user request.
The robot can implement the following tasks:
1. Move randomly in the environment, by choosing 1 out of 6 possible target positions:
   [(-4,-3);(-4,2);(-4,7);(5,-7);(5,-3);(5,1)], implementing a random position service.
2. Directly ask the user of the next target position (checking that the position is one of the possible six)
   and reach it
3. Start following the external walls
4. Stop in the last position

## Content of Package
### Nodes
1. (control)
* ROS publisher: publishing the robot speed.
* ROS subscriber: subscribing for robot status.
* ROS client: calling (user_interface),(move_random),(move_target),(wall_following) services.

2. (user_interface)
* ROS server: service server replys to the client with a option number.

3. (move_random)
* ROS server: service server, execute to move robot randomly.
* ROS publisher: publishing the robot target position.
* ROS subscriber: subscribing for robot position.
* ROS client: calling (random_pos) and receive a random target.

4. (move_target)
* ROS server: service server, executed to move robot to a target.
* ROS publisher: publishing the robot target position.
* ROS subscriber: subscribing for robot position.

5. (random_pos)
* ROS server: service server, replys to the client with a random target.

6. (wall_following)
* ROS server: Service Server, executed to let the robot follow wall.
* ROS publisher: publishing the robot speed.
* ROS subscriber: subscribing for laser scan value.

### Topics
- /cmd_vel
  - Type: geometry_msgs/Twist
  - Publisher Node: control
  - Subscriber Node: /gazebo 
- /odom
  - Type: nav_msgs/Odometry
  - Publisher Node: /gazebo
  - Subscriber Node: move_random, move_target, /move_base
- /move_base/goal
  - Type: MoveBaseActionGoal
  - Publisher : move_random, move_target, /move_base
  - Subscriber: /move_base
- /move_base/status
  - Type: GoalStatusArray
  - Publisher : /move_base
  - Subscriber: control

### Custom Services
- /move_to_target
  - Client Node: control
  - Server Node: move_to_target
  - Type: SetBool
- /move_random
  - Client Node: control
  - Server Node: move_random
  - Type : SetBool
- /user_interference
  - Client Node: control
  - Server Node: user_interference
  - Type : Empty
- /reading_laser
  - Client Node: control
  - Server Node: wall_follower_switch
  - Type : SetBool
- /random_pos
  - Client Node: control
  - Server Node : random_pos
  - Type : Trigger

### Parameter Server
* des_pos_x: 	destination position on x coordinates
* des_pos_y:	destination position on y coordinates
* option :		number of option which is chosen by user

## Computational Graph
![Communication Graph](../master/myFolder/nodes_only.png)

## How to run the code
1. Git clone 'final_assignment' and 'slam_gmapping' package
git clone https://github.com/juri-khanmeh/final_assignment.git
git clone https://github.com/CarmineD8/slam_gmapping.git
2. Move the repositories to your workspace
3. Build the packages 'catkin_make'
4. Refresh 'rospack profile'
5. Roslaunch 
* roslaunch final_assignment simulation_gmapping.launch
* roslaunch final_assignment move_base.launch
* roslaunch final_assignment final_assignment.launch

## Robot Behaviour
Let's describe the role of each main nodes.

- (control)
  - change the state depending on user's request.

- (move_to_targe)
  - make the robot move to a specific position.

- (move_random)
  - let the robot move freely in the environment.

- (wall_follow_switch)
  - make the robot follow the external wall.

- (user_interface)
  - ask user to enter an option number.

- (random_pos)
  -  generate the random target position from [(-4,-3),(-4,2),(-4,7),(5,-7),(5,-3),(5,1)].

Let's see the process flow.
- (user_interference)
* ask the user to insert an option.
* if the inserted number was from 1 to 4 the number is assigned to global parameter 'option'.

- Case 1: Move Randomly
* (move_random) service is called. In which (random_pos) is called and generate a random position. The generated position is assigned into the global parameter 'destination_pos'
* (move_random) node sends the 'destination_pos' to move_base by publishing on move_base/goal
* (move_random) node keeps printing robot info unitl the goal is reached. 
* (control) node subscribes to move_base/status topic in order to know when the robot reaches its goal.
* if the target is reached, (move_random) stops printing info and (control) node calls (user_interference) service.

- Case 2: Move to Target
* (move_to_targe) service is called. It asks the user to enter a target position. 
* If the target matches a predefined position, it assigns the entered position into the global parameter 'destination_pos'.
* (move_random) node keeps printing robot info unitl the goal is reached. 
* (control) node subscribes to move_base/status topic in order to know when the robot reaches its goal.
* if the target is reached, (move_random) stops printing info and (control) node calls (user_interference) service.

- Case 3: Following Wall
* (wall_follow_switch) is called. The robot starts to follow walls.
* (user_interference) service is called. (But the robot keeps moving and following walls).

- Case 4: Stop
* (control) node simply publishes zero velocities on /cmd_vel topic. 
* (user_interference) service is called to give the user the possibility to choose another option. 

## Software Architechture
![Architechture Graph](../master/myFolder/software_architecture.png)


## System's Limitation and Possible Improvement
* This robot can reach only some specific positions (not any position in the space).
* We can not set a target with an orientation.
* The robot can not move backwards (which leads to take more time to turn in narrow spaces and corners)

* This robot can be improved by giving it the ability to rotate around itself instead of turning around. This feature can help the robot to change its orientation in narrow spaces.



