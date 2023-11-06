# setcmas_expe

## Repository for experimenting distributed control of multi robots

Example of use: 

		roslaunch setcmas_expe expe_ctrl_mono_robot.launch algo:=wn_nav

		roslaunch setcmas_expe expe_ctrl_multi_robots.launch nb_robots:=6 algo:=consensus

where the parameter algo:=xxx referes to a Python script from the setcmas_ctrl package
