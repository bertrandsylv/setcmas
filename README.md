# setcmas
A software stack for Simulation and Experiments for Teaching Control of Multi-Agent Systems. 

It is composed of three packages for implementation of control laws (setcmas_ctrl), tests in simulation (setcmas_simu) and experiments (setcmas_expe). 

This version has been tested with ROS Noetic, Python 3.

Documentation is work in progress. 
Contact: sylvain.bertrand@onera.fr



## setcmas_ctrl

Control algorithms for single and multi agent systems. Implementation in Python. Examples are provided in setcmas_ctrl/scripts. To run scripts from setcmas_ctrl/scripts/exercises, use option exercise:=True.

Examples of use:

		roslaunch setcmas_ctrl ctrl_mono_robot.launch algo:=wp_nav

		roslaunch setcmas_ctrl ctrl_mono_robot.launch algo:=traj_tracking

		roslaunch setcmas_ctrl ctrl_multi_robots.launch nb_robots:=8 algo:=consensus
		
		roslaunch setcmas_ctrl ctrl_multi_robots.launch nb_robots:=8 algo:=consensus exercise:=True

		roslaunch setcmas_ctrl evt_triggered_ctrl_multi_robots.launch nb_norobts:=4 algo:=consensus

		roslaunch setcmas_ctrl ctrl_multi_robots.launch nb_robots:=5 algo:=leader_follower_formation


## setcmas_simu

Simulation of multi agent systems with unicycle dynamics (scripts/virtual_robot.py). 
It includes 3D visualization with RViz (Turtlebot3 3D models), and scripts for CSV data generation and visualization (matplotlib).


Examples of use:

		roslaunch setcmas_simu simu_mono_robot.launch

		roslaunch setcmas_simu simu_multi_robots.launch nb_robots:=6

		roslaunch setcmas_simu simu_multi_robots.launch nb_robots:=3 rviz_gui:=False

Record data when simulation is runing with following command line:

		rosbag record -a
		
Stop record with Ctrl + c. Use following command lines to convert recorded bag file to CSV file and plot data:

		python3 bag_to_csv.py NAME_OF_BAG_FILE.bag
		
		python3 plot_from_csv.py nbOfRobots


## setcmas_expe

Scripts for experiments with ground mobile robots. Tested with Turtlebot3 robots and motion capture systems for real time pose measurements. 

Examples of use:

		roslaunch setcmas_expe expe_ctrl_mono_robot.launch algo:=wp_nav

		roslaunch setcmas_expe expe_ctrl_multi_robots.launch nb_robots:=6 algo:=consensus

where the parameter algo:=xxx referes to a Python script from the setcmas_ctrl package