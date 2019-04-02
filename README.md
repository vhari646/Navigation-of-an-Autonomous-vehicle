# Navigation-of-an-Autonomous-vehicle
Navigation of an Autonomous Vehicle in Matlab
The purpose of this project was to develop a motion planning strategy for an autonomous vehicle in a dynamic environment.
The project consists of driving an autonomous vehicle firstly in a simulated environment.
The project was implemented using the following steps:
  a. Dead Reckoning:  The kinematic model used for implementing the dead reckoning was the time discrete car model. The estimated pose of        the robot is stored as a variable cPose and in the Current_pose matrix.
  b. Obstacle avoidance: The obstacle avoidance was implemented using the Vector Field Histogram (VFH) method. It uses a 2D Cartesian
     histogram grid as a world model. It Builds a 2D histogram base on the real-time laser scan data. It measures the distance d between
     the vehicle and object and generates a certainty value (CV). Next it Maps the 2D histogram onto a 1D polar histogram. It takes 10          degrees as a sector to construct the map. It evaluates the sector to check if it's clear or obstructed by setting a threshold to the        filter of the CV of all sectors in the map. For our task it contains 27 sectors.
  c. Kalman Filtering: The vehicle's current pose is not accurate even after the implementation of the dead-reckoning. The turning of the        vehilce is not correct. The Kalman filter is used to make adjustments to the vehicle's current pose. 
  d. SLAM: Simultaneously Localization and Mapping (SLAM) is used to make a map of the environment and to use this map for localization. We      used the LidarSLAM functiion from Matlab to create the map of the environment. At initialization the range of the Lidar and the            occupancy map resolution is set. Lowering the map resolution makes the algorithm perform faster but the map will have a lower              resolution for the grid.
  e. Enterance Detection: The enterance detection is implemented using Hough Transform to track the walls of the corridor. We set a              threshold value to extract the interval in the data. These intervals are used to detect the enterance.

Credits:
Map Creator - Henrik Söderlund - henrik.soderlund@umu.se
Kalman Filetr - Quentin Berche - queb0003@umu.se
Obstacle Avoidance - Weiyao Xiao - wexi0011@student.umu.se


  
