# ub_gazebo

This repository provides code for launching multiple robots in Gazebo (classic, v11).  This was mostly just to document things for myself; if this info helps you, then I'll be thrilled.

---

## Pre-reqs

This documentation was written for a system with these properties:
- ROS Noetic running on Ubuntu 20.04
- Gazebo Classic, Version 11
- The user's ROS catkin workspace is `~/catkin_ws/`; this `ub_gazebo` package will be installed in `~/catkin_ws/src/ub_gazebo`.

Currently, there is support for 2 kinds of robots:  The turtlebot3 and Clearpath Robotics Husky.

### Install turtlebot3 packages
- Start with the main packages:
    ```
    sudo apt-get update
    sudo apt --yes install ros-noetic-turtlebot3-msgs
    sudo apt --yes install ros-noetic-turtlebot3
    ```

- Now install the simulation package (see https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/):
    ```
    cd ~/catkin_ws/src/
    git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
    cd ~/catkin_ws && catkin_make
    ```

- For convenience, save environment variables in `~/.bashrc`.  Use any text editor to open the file (e.g., `pico ~/.bashrc`).  Then, add the following line to `~/.bashrc`:
```
export TURTLEBOT3_MODEL=waffle_pi  # options are burger, waffle, waffle_pi
```

### Install Husky packages
- Start with the main packages.  See https://www.clearpathrobotics.com/assets/guides/noetic/ros/Drive%20a%20Husky.html
    ```
    sudo apt-get --yes install ros-noetic-husky-desktop
    sudo apt-get --yes install ros-noetic-husky-simulator 
    sudo apt-get --yes install ros-noetic-husky-navigation
    ```

- For convenience, save environment variables in `~/.bashrc`.  Use any text editor to open the file (e.g., `pico ~/.bashrc`).  Then, add the following lines to `~/.bashrc`:
    ```
    # Enable front/rear scanners on Husky:
    export HUSKY_LMS1XX_ENABLED=1
    export HUSKY_LMS1XX_SECONDARY_ENABLED=1

    # Enable Intel RealSense camera, and shift its location:
    export HUSKY_REALSENSE_ENABLED=1
    export HUSKY_REALSENSE_XYZ="0.35 0 0.01"
    ```
---

## Install `ub_gazebo` (this repo)

```
cd ~/catkin_ws/src
git clone https://github.com/optimatorlab/ub_gazebo.git
```

- It is also recommended to checkout a branch, using your GitHub username as the branch name:
    ```
    cd ~/catkin_ws/src/ub_gazebo
    git checkout -b YOUR_GITHUB_USERNAME_GOES_HERE
    ```
    
Now, compile the `ub_gazebo` package:
```
cd ~/catkin_ws
catkin_make
```

Finally, add the following to `~/.bashrc`:
```
# Update model/resource paths to let Gazebo know where to find some of our materials
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/catkin_ws/src/ub_gazebo/models
export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:~/catkin_ws/src/ub_gazebo/launch/media
export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:~/catkin_ws/src/ub_gazebo/worlds
```
---

## Usage/Examples

The code in this repo is primarily for launching multiple robots/models in a Gazebo world.  Two different approaches are described below:


### Option 1 - Use a single `.launch` file to spawn a world with one or more robots
See [`world_plus_huskies.launch`](launch/world_plus_huskies.launch)
```
roscd ub_gazebo/launch
roslaunch ub_gazebo world_plus_huskies.launch
```

See [`world_plus_turtlebots.launch`](launch/world_plus_turtlebots.launch)
```
roscd ub_gazebo/launch
roslaunch ub_gazebo world_plus_turtlebots.launch
```

- NOTE:  For some reason, Gazebo is complaining when we try to spawn both Huskies and Turtlebots.  (Specifically, the message is a variant of `[ WARN] [1711136096.606182878, 56.509000000]: Could not obtain transform from base_link to husky/0_tf/base_link. Error was "base_link" passed to lookupTransform argument source_frame does not exist.`.).

		
### Option 2 - Spawn a world with one file, spawn robots separately
This approach works well if you want to add robots over time (instead of all at once).  This also works well for adding robots programmatically (e.g., via a Python script).

You'll need (at least) two terminal windows:

#### Terminal 1 - Use a `.launch` file to spawn a world, but with no robots.  
```
roscd ub_gazebo/launch
roslaunch ub_gazebo world_only.launch
```
The `world_only.launch` file can be edited to spawn different worlds.
    
#### Terminals 2, 3, 4, ... - Start simulated robots on demand.  
Here are two ways to do this:
- Using one terminal window per `roslaunch` command, spawn the robot of your choice:
    - `roslaunch ub_gazebo husky_spawn.launch robot_namespace:=husky/3 x:=1.3 y:=2.1 z:=0.0 yaw:=1.57 webcam_pitch:=0.0`
    - `roslaunch ub_gazebo turtlebot_spawn.launch robot_namespace:=tb/0 x:=2.3 y:=2.1 z:=0.0 yaw:=1.57`
    - Just make sure each robot has a unique `robot_namespace`.
- Alternatively, you can edit the [`robot_launcher.py`](scripts/robot_launcher.py) script and run it:
    ```
    roscd ub_gazebo/scripts
    python3 robot_launcher.py
    ```
    This will launch several Huskies or Turtlebots at once.
    
--- 

## Coming Soon

The following improvements are planned:
- [ ] Improved documentation
- [ ] Bug fixes for SOAR world
- [ ] Tutorial on how to add individual objects (trees, people, cones, ArUco tags, etc.) via Python
- [ ] Support for drones
    - PX4
    - Clover





