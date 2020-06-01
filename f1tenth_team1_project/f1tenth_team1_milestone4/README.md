Final Race:
No new additional dependencies. Find the code for the final race in the folder: f1tenth_team1_milestone4/final_race

How to run:

1) Clone directory onto local machine 
2) Copy f1tenth_team1_milestine4/final_race folder into the catkin_ws/src directory.
3) Change the address of the csv folder in the file (in 2 places: line 128 and line 146) - final_race/src/rrt.cpp
4) catkin_make and source files
5) roslaunch final_race team1_rrt.launch

PFM:
This folder contains code for obstacle dependent gaussian potential field. Follow the same commands as final_race package. 
In step 4, change command to : roslaunch pfm pfm.launch

TrajOpt:
This folder contains MATLAB code for trajectory optimization using CMAES. Curve fitting toolbox must be installed. 
Only run the file "trajectory_opt.m" from within the TrajOpt folder.
