# DRIMS2 Simulation Environment

## First steps

0. Update the docker repo `git pull DRIMS2_Docker` (**⚠️ Even you already cloned this repo, be sure to pull.**)
1. Start the [docker](https://github.com/AIRLab-POLIMI/DRIMS2_Docker) :  `./start.sh`
2. Compile and source the `drims_ws` workspace
```bash
cd drims_ws/
catkin_make
source devel/setup.bash
```

## Gazebo Simulation


To run the simulation:

1. Load Gazebo environment and robot simulation:
```bash
roslaunch drims_dice_demo arm_gazebo.launch robot:=yumi # or robot:=gofa
```
2. Launch low-level ROS Services Server for robot planning and control:
```bash
roslaunch abb_wrapper_control launchControlServer.launch robot:=yumi # or robot:=gofa
```

3. Launch high-level ROS Services Server for robot planning and control:
```bash
roslaunch abb_wrapper_control launchTaskServer.launch robot:=yumi # or robot:=gofa
```

4. Spawn the dice in simulation:
```bash
roslaunch drims_dice_demo spawn_dice.launch
```

**NOTE:** Each command should run in its own terminal

## Real robot

**⚠️ To use the real robot ask a tutor.**

Steps to use the real robot (both Yumi or Gofa):
- You need an Ubuntu machine
- Connect your notebook to the physical robot via Ethernet
- Set your IP as Static IP: (`192.168.125.100`, Netmask: `255.255.255.0`, Empty gateway)
- Test the connection with: `ping 192.168.125.1` (from outside Docker)

**⚠️ NOTE:** If you are using the Gofa robot, before launching the real robot, you must run the following command:
```bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib
```

Now you are ready to work:
1. Launch the real robot:
```bash
roslaunch drims_dice_demo real_robot.launch robot:=yumi # or gofa
```

2. Open another terminal and connect to the docker `./connect.sh`, and launch low-level ROS Services Server for robot planning and control:
```bash
roslaunch abb_wrapper_control launchControlServer.launch robot:=yumi # or robot:=gofa
```

3. Open another terminal and connect to the docker `./connect.sh`, and launch high-level ROS Services Server for robot planning and control:
```bash
roslaunch abb_wrapper_control launchTaskServer.launch robot:=yumi # or robot:=gofa
```

## Example nodes

You can test the functions with the following commands (⚠️**Keep in mind the notes that are given below**):

**Python:**

```bash
rosrun drims_template take_the_dice_example.py
```

**C++:**

```bash
roslaunch drims_template test_example.launch
```
This launcher runs the script `test_example.cpp` located in the `DRIMS_template` ROS package.

**⚠️ NOTE**: remember to press 'next' in the RvizVisualToolsGui window to execute the motion on the robot.
(Check the terminal where you launched the `launchControlServer.launch` to visualize the previous blue message)

**⚠️ NOTE 2**: the example assumes the dice localization is active. If you want to test the nodes with the real robots and without the vision, you can fake the dice localization with:
```bash
roslaunch drims_template test_spawn_dice.launch
```

Both the examples present a basic use of the services to move the robot:
- `/plan_and_execute_pose`: plan a collision-free joint trajectory to a given goal Cartesian pose and execute it.
- `/plan_and_execute_slerp`: plan a collision-free joint trajectory by using SLERP interpolation to a given a goal Cartesian pose and execute it.
- `/plan_and_execute_joint`: plan a collision-free joint trajectory to a given a Joint goal and execute it.
- `/open_gripper`: open the gripper.
- `/close_gripper`: close the gripper.

The following gives the ROS service definition, located in the `srv` folder of `abb_wrapper_msgs` ROS package:

## Available ROS services to move the robot

**Service structure `abb_wrapper_msgs/plan_and_execute_pose`**
```
# == Request ==
geometry_msgs/Pose goal_pose
bool is_relative
---
# == Response ==
bool success
string message
```

**Service structure `abb_wrapper_msgs/plan_and_execute_joint`**
```
# == Request ==
float64[] joint_goal
---
# == Response ==
bool success
string message
```

**Service structure `abb_wrapper_msgs/plan_and_execute_slerp`**
```
# == Request ==
geometry_msgs/Pose goal_pose
bool is_relative
---
# == Response ==
bool success
string message
```

**Service structure `abb_wrapper_msgs/open_gripper` and `abb_wrapper_msgs/open_gripper`**
```
bool in_flag
---
bool out_flag
string message
```

<!--
%## Dice manipulation task
%
%Now, you can start to create your own task for dice manipulation.%
%
%For the **C++** version, you can modify this ROS node (`task_dice.cpp`) between lines 154-164 to create your sequence of actions to perform the dice manipulation.
%Remember that you can launch the previous ROS node by using the following command:
%```bash
%roslaunch drims_template task_dice.launch
%```
-->




