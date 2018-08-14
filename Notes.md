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
- The [gmapping](http://wiki.ros.org/gmapping) package uses the filtered odometry data from the EKF (robot_localization pkg) along with the lidar scan data in order to build a dynamic map (occupancy grid) around the robot
- The [navigation stack](http://wiki.ros.org/navigation?distro=kinetic) works by creating two separate costmaps, and two separate planners
    - The Global Costmap takes in a map (Occupancy Grid) from the gmapping package and uses it to dynamically build a costmap over time, designating areas that are free, unknown, or occupied (designated by values between 0-255)
    - The Global Planner takes this data and plans a global path using a `.mprim` file generated in by matlab that describes that kinetmatic constraints of the car. Detailed instructions can be found [here](http://sbpl.net/node/52)
    - The local costmap creates a smaller costmap that is frequently updated, directly from lidar data (doesn't use gmapping and is always centered on the robot's current position)
    - The local planner (in our case using the teb_local_planner as a plugin to move_base), uses the local costmap along with the global plan to create a short term path and series of future poses along with the neccessary linear and angular velocities (geometry_msgs/Twist) to reach the goal
- Drivers
    - [Razor IMU](https://github.com/KristofRobot/razor_imu_9dof)
    - [RPLidar A2](https://github.com/Slamtec/rplidar_ros)
    - [ZED Camera](https://github.com/willdzeng/zed_cpu_ros) (Only outputs raw video streams, depth sensing requires Nvidia CUDA and a different driver)