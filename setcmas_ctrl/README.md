# setcmas
A software stack for Simulation and Experiments for Teaching Control of Multi-Agent Systems. 

It is composed of three packages for implementation of control laws (setcmas_ctrl), tests in simulation (setcmas_simu) and experiments (setcmas_expe). 

This version has been tested with ROS Noetic, Python 3.

Documentation is work in progress. 
Contact: sylvain.bertrand@onera.fr

Associated paper: 
S. Bertrand, "SETCMAS: An easy-to-use software stack to facilitate Simulations and
Experiments in Teaching Control of Multi-Agent Systems", European Control Conference, Stockholm, Sweden, 2024.



## setcmas_ctrl

Control algorithms for single and multi agent systems. Implementation in Python. Examples are provided in setcmas_ctrl/scripts. To run scripts from setcmas_ctrl/scripts/exercises, use option exercise:=True.

Examples of use:

		roslaunch setcmas_ctrl ctrl_mono_robot.launch algo:=wp_nav

		roslaunch setcmas_ctrl ctrl_mono_robot.launch algo:=traj_tracking

		roslaunch setcmas_ctrl ctrl_multi_robots.launch nb_robots:=8 algo:=consensus
		
		roslaunch setcmas_ctrl ctrl_multi_robots.launch nb_robots:=8 algo:=consensus exercise:=True

		roslaunch setcmas_ctrl evt_triggered_ctrl_multi_robots.launch nb_norobts:=4 algo:=consensus

		roslaunch setcmas_ctrl ctrl_multi_robots.launch nb_robots:=5 algo:=leader_follower_formation
