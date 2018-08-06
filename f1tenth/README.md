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








      		    # Optimization

                no_inner_iterations: 6
                no_outer_iterations: 5
                optimization_activate: True
                optimization_verbose: False
                penalty_epsilon: 0.1
                weight_max_vel_x: 10
                weight_max_vel_theta: 1
                weight_acc_lim_x: 1
                weight_acc_lim_theta: 1
                weight_kinematics_nh: 1000
                weight_kinematics_forward_drive: 1  #1
                weight_kinematics_turning_radius: 15 #1
                weight_optimaltime: 60
                weight_obstacle: 50
                weight_dynamic_obstacle: 1 # not in use yet
                weight_viapoint: 50
                # Homotopy Class Planner

                enable_homotopy_class_planning: True #change to False 1003
                enable_multithreading: True
                simple_exploration: False
                max_number_classes: 4
                selection_cost_hysteresis: 1.0
                selection_obst_cost_scale: 1.0
                selection_alternative_time_cost: False
                roadmap_graph_no_samples: 15
                roadmap_graph_area_width: 5
                h_signature_prescaler: 0.5
                h_signature_threshold: 0.1
                obstacle_keypoint_offset: 0.1
                obstacle_heading_threshold: 0.45
                visualize_hc_graph: False
                
         rosbag filter manytopics.bag fewtopics.bag 'topic=="/topic1","/topic2","/topic3"' Bag to CSV/Text
         
         <node name="hallway_data" pkg="rosbag" type="play" output="screen" args="--clock -r 1 /home/aswerdlow/Documents/logs/_2018-08-06-12-09-07.bag"/>````