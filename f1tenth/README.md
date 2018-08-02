# f1tenth project

-r 1 -s 2 -u 100

<node name="scans_demo" pkg="rosbag" type="play" output="screen"
		  args="-r 1 --clock  /home/aswerdlow/Documents/logs/_2018-07-26-15-49-45.bag "/>

	<node pkg="race" type="L1_controller_v3" respawn="false" name="L1_controller_v3" output="screen">
		<!-- L1 -->
		<param name="Vcmd" value="1.0"/> <!-- speed of car m/s -->
		<!-- ESC -->
		<param name="baseSpeed" value="1425"/> <!-- pwm for motor constant speed, 1480: stop, 1440: ~0.5m/s, 1430: ~1.5m/s -->
		<!-- Servo -->
		<param name="baseAngle" value="90.0"/> <!-- the middle pos of servo, for tt02: 87, for hsp: ? -->
		<param name="AngleGain" value="-3.5"/> <!-- for tt02: >0, for hsp: <0 -->

		<!-- remap from="/odometry/filtered" to="odom" / -->
		<remap from="/move_base_node/NavfnROS/plan" to="/move_base/NavfnROS/plan"/>
	</node>



	 try
      {
    	geometry_msgs::Pose odom_goal;
    	tf_listener.transformPose("map", ros::Time::now(), *odom_msg, "odom" , odom_goal);
    	//std::cout << odom_goal < endl;

      }
      catch(tf::TransformException &ex)
      {
    	ROS_ERROR("%s",ex.what());
    	ros::Duration(1.0).sleep();
      }
      tf::TransformListener tf_listener;

      -r 1 -u 32 --clock -l

      	<param name="/use_sim_time" value="true"/>
      	<node name="scans_demo" pkg="rosbag" type="play" output="screen"
      		  args="rosbag --clock -l /home/aswerdlow/Documents/logs/_2018-08-01-13-09-18.bag"/>