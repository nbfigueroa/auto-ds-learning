# dsltl

This package includes code to segment trajectories of a multi-step demonstrated task. The tasks considered are those that can be defined as a sequence of goal-oriented sub-tasks. In this work, each sub-task is characterized by a a Dynamical System (DS) Motion Policy and an attractor (representing the policy goal) found inside each Action Proposition (AP) region. Segmentation points are inferred tracking state changes in the pre-defined AP regions of the task. 

This approach has been used to learn the following multi-step tasks:
- **Mitsubishi pick-scan-check-place task:** The robot should grasp electronic components from trays and go through a sequence of pick-and-place steps to scan and check the quality of the part. 
- **Franka cooking task:** The robot should scoop ingredients from two distinct bowls (emulated as marbles) and transport them and release them in a mixing bowl. 
<!-- - **Franka table setting task:** The robot grasps plates and cutlery from a dish rack/stand and place them in the demonstrated locations.  -->

The scripts described below were used to process all of these tasks and should be a starting point for any new task. 

We assume that before starting these steps you have run the scripts in [rosbag_to_mat](https://github.com/nbfigueroa/rosbag_to_mat) to extract the trajectories from the ROSbags into MATLAB data structures required for these steps.

## Step 1: Trajectory Segmentation for Sequence of DS learning 
The segmentation step is performed in the following matlab scripts: 
- ``mitsubishi_segment_trajectories.m``
- ``franka_cooking_segment_trajectories.m`` 
The structure of these MATLAB scripts is identical, only adhoc changes based on experiment/workspace differences. 

#### First Code Block (Data Loading)
Load the trajectories and visualize the AP regions that will be used for segmentation (convex hull around workspace objects), which should appear like this:
<p align="center">
  <img src="https://github.com/yanweiw/dsltl/blob/main/figs/mitsubishi_raw_trajectories.png" width="349x"><img src="https://github.com/yanweiw/dsltl/blob/main/figs/franka_cooking_all_trajectories.png" width="425x">
</p>

<p align="center">    
    Left: Mitsubishi pick-scan-check-place task, Right: Franka Simple Cooking Task (2 bowls)
</p>

In these plots we provide a visualization of the recorded trajectories, tables/stands and the AP regions for each task.   
**Note: The colors of the trajectories correspond to each continuous demonstration.**

#### Second Code Block (Continuous Trajectory Segmentation)
This code block segments the trajectories based on the state-change tracking wrt. those AP regions. 
<p align="center">
  <img src="https://github.com/yanweiw/dsltl/blob/main/figs/mitsubishi_segmented_trajectories_all.png" width="349x"><img src="https://github.com/yanweiw/dsltl/blob/main/figs/franka_cooking_segmented_trajectories_all.png" width="475x"> 
</p>

**Note: Now the colors of the trajectories represent the segments for each sub-task.**

We finalize with N_{seg} number of segments/attractors/DS/trajectory-clusters to represent each multi-step task.


#### Third Code Block (Segmented Trajectory Pre-Processing)
**Step 3:** pre-process and smoothen the N_{seg} segmented trajectory clusters to prepare for the DS learning stage.

- **Mitsubishi pick-scan-check-place task**: The trajectories for N_{seg}=6 actions have been clustered, processed and smoothed for DS learning: 
<p align="center">
  <img src="https://github.com/yanweiw/dsltl/blob/main/figs/mitsubishi_DS_s1_traj.png" width="300x"><img src="https://github.com/yanweiw/dsltl/blob/main/figs/mitsubishi_DS_s2_traj.png" width="300x"><img src="https://github.com/yanweiw/dsltl/blob/main/figs/mitsubishi_DS_s3_traj.png" width="300x"> 
  <img src="https://github.com/yanweiw/dsltl/blob/main/figs/mitsubishi_DS_s4_traj.png" width="300x"><img src="https://github.com/yanweiw/dsltl/blob/main/figs/mitsubishi_DS_s5_traj.png" width="300x"><img src="https://github.com/yanweiw/dsltl/blob/main/figs/mitsubishi_DS_s6_traj.png" width="300x"> 
</p>


- **Franka cooking task:** The trajectories for N_{seg}=3 actions have been clustered, processed and smoothed for DS learning: 
<p align="center">
  <img src="https://github.com/yanweiw/dsltl/blob/main/figs/franka_cooking_DS_s1_clustered_trajectories.png" width="306x"><img src="https://github.com/yanweiw/dsltl/blob/main/figs/franka_cooking_DS_s2_clustered_trajectories.png" width="302x"><img src="https://github.com/yanweiw/dsltl/blob/main/figs/franka_cooking_DS_s3_clustered_trajectories.png" width="300x"> 
</p>


## Step 2: DS Motion Policy Learning from Segmented Trajectories 
Once trajectories are segmented into N_{seg} datasets, for each trajectory cluster we learn a DS motion policy following the LPV-DS approach proposed in [Figueroa and Billard, 2018](http://proceedings.mlr.press/v87/figueroa18a.html). All the necessary code to learn these LPV_DS is included the directory ``./ds-libraries``. 

### Installation
To setup ds-libraries run: ``setup_code.m``

To test phys-gmm code (Bayesian Nonparametric GMM clustering from [Figueroa and Billard, 2018](http://proceedings.mlr.press/v87/figueroa18a.html)):    
```
cd ./ds-libraries/phys-gmm  
demo_loadData.m
```  
This will fit a GMM to the concentric circles dataset.

To test ds-opt code (non-convex SDP optimization from [Figueroa and Billard, 2018](http://proceedings.mlr.press/v87/figueroa18a.html)):   
```
cd ./ds-libraries/ds-opt 
demo_learn_lpvDS.m
```  
This will learn an LPV-DS on the S-shape dataset.

### Learning and Simulating the DS Sequences
The DS learning step is performed in the following matlab scripts: 
- ``mitsubishi_learn_sequenceDS.m``
- ``franka_cooking_learn_sequenceDS.m`` 
These scripts will learn N_{seg} DS with the given trajectories defined by the previous step. The structure of these MATLAB scripts is identical, only adhoc changes based on experiment/workspace differences. 

#### First Code Block
The sequence of DS is learned. You will visualize the trajectories and reproductions of the DS from a range of initial positions. Additionally, yaml and txt files with each of the DS parameters are automatically created and stored in the ``./models/`` directory. 

- **Mitsubishi pick-scan-check-place task**: The trajectories for N_{seg}=6 actions have been clustered, processed and smoothed for DS learning: 
<p align="center">
  <img src="https://github.com/yanweiw/dsltl/blob/main/figs/mitsubishi_DS_s1.png" width="300x"><img src="https://github.com/yanweiw/dsltl/blob/main/figs/mitsubishi_DS_s2.png" width="300x"><img src="https://github.com/yanweiw/dsltl/blob/main/figs/mitsubishi_DS_s3.png" width="300x"> 
  <img src="https://github.com/yanweiw/dsltl/blob/main/figs/mitsubishi_DS_s4.png" width="300x"><img src="https://github.com/yanweiw/dsltl/blob/main/figs/mitsubishi_DS_s5.png" width="300x"><img src="https://github.com/yanweiw/dsltl/blob/main/figs/mitsubishi_DS_s6.png" width="300x"> 
</p>


- **Franka cooking task:** The trajectories for N_{seg}=3 actions have been clustered, processed and smoothed for DS learning: 
<p align="center">
  <img src="https://github.com/yanweiw/dsltl/blob/main/figs/franka_cooking_DS_s1_clustered_trajectories.png" width="306x"><img src="https://github.com/yanweiw/dsltl/blob/main/figs/franka_cooking_DS_s2_clustered_trajectories.png" width="302x"><img src="https://github.com/yanweiw/dsltl/blob/main/figs/franka_cooking_DS_s3_clustered_trajectories.png" width="300x"> 
</p>


#### Second Code Block
Simulates the sequence of the N_{seg} learned DSs and creates a video of the animation if the following variable is set in the script:
```matlab
% Animate Sequence of DS
make_vid = 1;
```
<p align="center">
  <img src="figs/mitsubishi_trajectories_APregions.png" width="300x"><img src="figs/franka_cooking_trajectories_APregions.png" width="300x"> 
</p>

## Using a Learned DS as a Motion Policy for Robot End-Effector Control
Once you have verified that the DSs were learned correctly (and exhibit the desired behavior) we can use them as a motion policy to the control the end-effector of a real robot. There are several ways to accomplish this. These are listed below:
- [lpvDS-lib](https://github.com/nbfigueroa/lpvDS-lib): This repository contains a standalone C++ library that reads the parameters of the learned DS, in either yaml/txt format and can compile the library either with ROS-ified catkin_make or pure cmake compilation commands. This is the preferred library if you are NOT using ROS to control your robot and use/like C++. 
- [ds_motion_generator](https://github.com/nbfigueroa/ds_motion_generator) **[Preferred]**: This repository is a ROS package that includes nodes for DS motion generation of different types (linear DS, lpv-ds, lags-ds, etc,). It depends on the standalone [lpvDS-lib](https://github.com/nbfigueroa/lpvDS-lib) library and uses yaml files to load the parameters to ros node. This node generates the desired velocity from the learned DS (given the state of the robot end-effector) and also filters and truncates the desired velocity (if needed). The node also generates a trajectory roll-out (forward integration of the DS) to visualize the path the robot will follow -- this is updayed in realtime at each time-step. 
- [ds-opt-py](https://github.com/nbfigueroa/ds-opt-py): This is an experimental python package that includes a python library to read parameters in yaml format for the execution of the lpv-DS, ROS integration is yet to be done, but should be straightforward. However, filtering and smoothing should be done separately. 
 

