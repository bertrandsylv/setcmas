# setcmas
A software stack for Simulation and Experiments for Teaching Control of Multi-Agent Systems. 

It is composed of three packages for implementation of control laws (setcmas_ctrl), tests in simulation (setcmas_simu) and experiments (setcmas_expe). 

This version has been tested with ROS Noetic, Python 3.

Documentation is work in progress. 
Contact: sylvain.bertrand@onera.fr

Associated paper: 
S. Bertrand, "SETCMAS: An easy-to-use software stack to facilitate Simulations and
Experiments in Teaching Control of Multi-Agent Systems", European Control Conference, Stockholm, Sweden, 2024.


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
