# setcmas
A software stack for Simulation and Experiments for Teaching Control of Multi-Agent Systems. 

It is composed of three packages for implementation of control laws (setcmas_ctrl), tests in simulation (setcmas_simu) and experiments (setcmas_expe). 

This version has been tested with ROS Noetic, Python 3.

Documentation is work in progress. 
Contact: sylvain.bertrand@onera.fr

Associated paper: 
S. Bertrand, "SETCMAS: An easy-to-use software stack to facilitate Simulations and
Experiments in Teaching Control of Multi-Agent Systems", European Control Conference, Stockholm, Sweden, 2024.


## setcmas_expe

Scripts for experiments with ground mobile robots. Tested with Turtlebot3 robots and motion capture systems for real time pose measurements. 

Examples of use:

		roslaunch setcmas_expe expe_ctrl_mono_robot.launch algo:=wp_nav

		roslaunch setcmas_expe expe_ctrl_multi_robots.launch nb_robots:=6 algo:=consensus

where the parameter algo:=xxx referes to a Python script from the setcmas_ctrl package