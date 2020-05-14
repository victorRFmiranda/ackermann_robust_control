# ackermann_robust_control
This is a ROS package contain a Robust controller designed to be robust to parametric uncertainties and external disturbances!
Considering a nonlinear Ackermann model: uncertainties addressed are ground friction and mass of vehicle; road inclination is considered as external disturbance.


### Compiling
1. Install the EKF package Robot Pose EKF
```
$ sudo apt-get install ros-$ROS_DISTRO-robot-pose-ekf -y
```

2. Clone this repository in your ROS worspace
```
$ cd ~/catkin_ws/src
$ git clone https://github.com/victorRFmiranda/ackermann_robust_control
```

3. Compile
```
$ catkin build or catkin_make
```

### Simulator Usage

There are two control methods that can be use to execute automous system: PID and Backstepping.

In this package, there is a launch file for each controller type.

For use in the CoppeliaSim, first open the simulator and make sure that ROS is already running:
Type this in a first terminal linux:
```
$ roscore
```
and start the CoppeliaSim in another terminal.

If everything is ok, the topic `\tf` will appear when you type the followin in another terminal:
```
$ rostopic list
```

Open a scenario of your preference. The scenes are inside of this package folder `sim_models_and_scenarios`.

After this, it's necessary run the ROS launch that correspond to the desired controller:

1. PID:
```
$ roslaunch ackermann_robust_control pid.launch
```

2. Backstepping:
```
$ roslaunch ackermann_robust_control backstepping.launch
```

Now, some topics will appear. The trajectory must publish the velocity and orientation in the topcis `/cmd_vel` and `/yaw_angle`. The information topics are `\odom` and `/robot_pose_ekf/odom_combined`.

If you have a .txt file with orientation and velocity date of a trajectory, it's possible to use the trajectory code:
```
$ rosrun ackermann_robust_control trajectory.py
```

Remenber of change the name of the .txt file in config.yaml.

### Parameters

The controller parameters are in `config/config.yaml`, change carefully.

In this file is possible to change PID and Backstepping gains, parameters of the vehicle, frequency of publication of some topics, name of the trajectory file, among others.

### Contact
victormrfm@gmail.com
