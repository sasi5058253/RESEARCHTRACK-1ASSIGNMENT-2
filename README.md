# RT-1 ASSIGNMENT-2

# BY S5058253 SASI KUMAR UPPATALA

# Descrption of the Assignmnet: 

The assignment is done in a ROS architecture for controling a mobile robot in the Gazebo environment.The software relies on the move_base and gmapping packages for localizing the robot and plan its motion. The program acquires the user's request, and lets the robot execute one of the pre-defined behaviors accordingly, along with Simulataneous Localization and Mapping(SLAM),path planning and collision avoidance.specially,Gazebo and Rviz used for the simulation of the assignment,whereas Gazebo is the 3d simulator for ROS and rviz,abbreviation for ROS visualization,is a powerful 3D visualization tool for ROS.It allows the user to view the simulated robot model, log sensor information from the robot's sensors, and replay the logged sensor information.

The program is able to acquire the requests of user input on the following states:

1.Robot moves randomly in the environment, by choosing 1 out of 6 possible target positions: [(-4,-3);(-4,2);(-4,7);(5,-7);(5,-3);(5,1)].

2.And asks the user for the next target position, checking that the position is one of the possible six target positions, and the robot reaches it.

3.Robot starts following the external walls.

4.Robot stops in the last position.
{If the robot is in state 1 or 2, the system waits until the robot reaches the position in order to switch to state 3 or 4}.

# how the Mobile Robot is controlled:

The final_assignment package contains the scripts,launch files and other dependencies used to simulate the 3D environment and move the robot in it.The simulation_gmapping.launch file launches the house.world file environment.The importatant node final_user_req.py contains python codes which is responsible for the mobile robot simulation.

The simulation is done in following steps:

1.For the first step, final_user_req.py node requests my_srv for a random target position between the range of 1 to 6.Then,the main node publishes the target positions to */move_base/goal* and check thes the status of goal by subscribing to the topic */move_base/status*.When the robot reaches the target and the status of robot is displayed, the main node requests the user to input again.

2.For the second step, the user chooses one out of six possible target positions and publishes it to */move_base/goal*.

3.For the third step,the wall_follower service is generated through initialization of a service client to allow the robot to follow the external walls.

4.For the fourth step, the node stops all actions and stops the robot by publishing commands of zero velocity in topic */cmd_vel*.

In steps 3 and 4,The interface also allows the user to enter the same or different request at any point in this state.


# my_srv(server):

The server package my_srv contains the Cpp file finalassignment_server.cpp which contains the source code for generating random integer within a specified range and advertising it over the node /second assignment. It uprovides a requests with two integers namely min and max, and returns one random integer target_index within this range in response.
to reach assignment https://github.com/sasi5058253/RESEARCHTRACK-1ASSIGNMENT-2

# Simulation of the assignment is done by follwing commands:


1.In the command terminal, launch Gazebo and rviz by executing the following command:

    roslaunch final_assignment simulation_gmapping.launch
2.In a new command line terminal, run the following command:

    roslaunch final_assignment move_base.launch
3.In a new command line terminal, run the following command:

    rosrun final_assignment wall_follow_service_m.py
4.In a new command line terminal, run the following command:
     
     rosrun my_srv finalassignment_server
5.In a new command line terminal, run the following command:
      
      rosrun final_assignment final_user_req.py
6.To display the computational graph of the system,run the following command:

    rosrun rqt_graph rqt_graph
