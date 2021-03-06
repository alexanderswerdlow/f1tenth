### f1tenth notes
- In order to access data from usb devices, ensure that the device is plugged in and you are accessing the correct port (Use `ls /dev/tty*` to check, you should look for `/dev/tty/USB*` or `/dev/tty/ACM*`)
- You also need to add your user to the dialout group by running `sudo adduser example_username dialout`
- Need to make python files executable, ex. `chmod +x example.py`
- Building (using catkin_make) for the first time takes a long time due to building costmap
    - The navigation and teb_local_planner packages are cloned in this repo to allow for editing and debugging, but this does cause builds to take far longer than when using the compiled binaries
- Use the tag `output="screen"` when calling ROS Nodes from a launchfile to pipe the output to the console
- Modifying the accepted data in the EKF Param file (ex. `odom0_config` param), can cause the filter to not output certain odometry values
- Adding code below to your `CMakeLists.txt` file in `~/catkin_ws/src` allows IDEs such as clion load all the CMakelists files below the current directory which allows for helpful autocorrect, error-checking, and go-to-definiton features
    ``` 
    FOREACH(subdir ${SUBDIRS})
    ADD_SUBDIRECTORY(${subdir})
    ENDFOREACH()```
- Adding `include_directories(~/catkin_ws/devel/include)` to the same file will allow IDEs to view any custom message files that your packages create
    - Add these lines directly above the `catkin_workspace()` line at the end
- The [robot_localization](http://wiki.ros.org/robot_localization) package provides a simple interface for an EKF, or UKF and in our case, uses data from the rf2o_laser_odometry package and the IMU to estimate the current state (in the odom frame, of type `nav_msgs/Odometry`)
    - In some cases when certain inputs are set to false for the UKF, the package will output no useful odometry data (All zeroes) without issuing a warning message. Setting all values to true in the odom/imu/twist config can reveal if this is an issue.
- The [gmapping](http://wiki.ros.org/gmapping) package uses the filtered odometry data from the EKF (robot_localization pkg) along with the lidar scan data in order to build a dynamic map (occupancy grid) around the robot
- The [navigation stack](http://wiki.ros.org/navigation?distro=kinetic) works by creating two separate costmaps, and two separate planners
    - The Global Costmap takes in a map (Occupancy Grid) from the gmapping package and uses it to dynamically build a costmap over time, designating areas that are free, unknown, or occupied (designated by values between 0-255)
    - The Global Planner takes this data and plans a global path using kinematic constraints and a goal state (x, y, theta)
    - The local costmap creates a smaller costmap that is frequently updated, directly from lidar data (doesn't use gmapping and is always centered on the robot's current position)
    - The local planner (in our case using the teb_local_planner as a plugin to move_base), uses the local costmap along with the global plan to create a short term path and series of future poses along with the neccessary linear and angular velocities (geometry_msgs/Twist) to reach the goal
- [SBPL Global Planner](http://wiki.ros.org/sbpl_lattice_planner)
    - Uses a `.mprim` file generated in by matlab that describes that kinetmatic constraints of the car. Detailed instructions can be found [here](http://sbpl.net/node/52)
        - The `.mprim` file specifies possible next states and uses a weight to specify the preferred transformation 
    - Two packages are required to use SBPL. sbpl_latice_planner allows move_base to interface with the sbpl library. The sbpl package contains the updated library and all of the planner logic.
- [TEB Local Planner](http://wiki.ros.org/teb_local_planner)
    - Info about the robot footprint params can be found [here](http://wiki.ros.org/teb_local_planner/Tutorials/Obstacle%20Avoidance%20and%20Robot%20Footprint%20Model)
- Drivers
    - [Razor IMU](https://github.com/KristofRobot/razor_imu_9dof)
    - [RPLidar A2](https://github.com/Slamtec/rplidar_ros)
    - [ZED Camera](https://github.com/willdzeng/zed_cpu_ros) (Only outputs raw video streams, depth sensing requires Nvidia CUDA and a different driver)
    - [Teensy](https://github.com/mlab-upenn/f1tenthpublic/tree/master/code/Teensy%20Firmware) To change message files for the teensy, you need to add a msg file in `f1tenth/msg`, update `add_message_files` in CMakeLists, then follow the instructions in the f1tenth teensy readme. You must cmake, and rebuild the rosserial libraries folder.
- [Launch files](http://wiki.ros.org/roslaunch)
    - An sh file is used when running code on the robot so that the imu, lidar, and teensy can startup before recording data or running navigation   
- [Paramaters](http://wiki.ros.org/rosparam#YAML_Format)
    - [.yaml files](http://wiki.ros.org/rosparam#YAML_Format) can be used to define ros parameters.
        - Indents restrict the scope of the parameters, only allowing access to node handles whose name matches the name at the top of the indented block
- [Rosbag](http://wiki.ros.org/rosbag/Commandline)
    - To create a new bag file from an existing one up can use the install.sh script
    - For faster updates, go to update settings and change main server to mirrors.ocf.berkeley.edu
    - The script will disable IPv6 (also speeds up updates), updates and upgrades all packages, installs the full desktop version of ROS Kinetic, creates a catkin workspace in `~/catkin_ws`, clones the f1tenth repo, adds shortcuts to the user bash profile, edits the hosts file, runs catkin_make, then installs VS Code and the python & cpp extensions (along with pip and pylint), and opens VS Code
    - Only use the script on Ubuntu Xenial (16.04)
- An example of a working tf tree is shown in the `other` dir
- Bash profile
    - Below are some useful commands to add to your bash profile
    - Source ROS in every terminal window (line 1 & 2)
    - define the ROS Master location and ROS IP (line 3 & 4) to point to the robot
    - Define the ROS Master as local (line 5 & 6) to not connect to ROS on the robot if desired
    - Define an alias to easily view the tf tree (line 7)
    - Define an alias to install dependencies and runs `catkin_make` (line 8)
    - Define an alais to run `catkin_clean` and `catkin_make` (line 9)
    - Define an alias to easily ssh into the robot with X11 display server enabled (line 10)
        ```source /opt/ros/kinetic/setup.bash
       source ~/catkin_ws/devel/setup.bash
       export ROS_MASTER_URI=http://192.168.1.1:11311
       export ROS_IP=192.168.1.110
       #export ROS_HOSTNAME=localhost
       #export ROS_MASTER_URI=http://localhost:11311
       alias tf='cd /var/tmp && rosrun tf view_frames && evince frames.pdf &'
       alias cm='cd ~/catkin_ws && rosdep install --from-paths -i -y src -r && catkin_make && cd -'
       alias cclean='cd ~/catkin_ws && catkin_make clean && catkin_make && cd -'
       alias log='ssh -X cyber-physical@cyber-physical'
        ```
- Useful Repos
    - https://github.com/mrsd16teamd/loco_car/
    - https://github.com/Hypha-ROS/hypharos_racecar
    - https://github.com/mlab-upenn/f1tenthpublic/tree/master/code
    - https://github.com/tysik/obstacle_detector
    - https://github.com/ceezeh/UnmannedNavigationSystem