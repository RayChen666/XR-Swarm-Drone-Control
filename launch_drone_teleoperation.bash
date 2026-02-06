#!/bin/bash

export DISABLE_AUTO_TITLE="true"

session="TeleControl"

tmux new-session -d -s $session -n home


#get robot number
number=$1


echo $number

##################################
# case 1: Simulation + Rviz (select 1 in the param file)
#case 2: Simulation + (Holo or Unity or Webxr), in the case of webxr the script ./launch_NYU_FRL needs to be launched (select 2 in the param file)
# case 3: Simulation + Haptic (select 3 in the param file)
# case 4: Real Rviz + Holo + Webxr  (select 4 in the param file)
# case 5: Real + Haptic
#################################


read -p "Choose the desired launching session:
        - Drone Teleoperation in Simulation: press 1
        - Drone Teleoperation in Simulation with Holo: press 2
        - Drone Teleoperation in Simulation with Haptic: press 3
        - Drone Teleoperation in Real Rviz or Holo: press 4
        - Drone Teleoperation in Real Rviz with Haptic: press 5
        - Replay Rosbag User Case Study: press 7
        !!!! [Remember] in case 2 launch Vicon node and fkie separately" n1


read -p "Continuing in 1.0 seconds..." -t 1.0


#### LAUNCH SIMULATION + ROSBAG for mapping
if [[ "$n1" -eq 1 ]]; then

  STR1="ros2 bag play "
  STR2="gamelab_map3" #$rosbag 
  STR3=" --remap /race10/nvblox_node/static_occupancy:=/quadrotor/nvblox_node/static_occupancy"
  STR="$STR1$STR2$STR3"
  

   tmux split-window -t $session:0.0 -v -p 33  
   tmux split-window -t $session:0.0 -v -p 33 
   tmux split-window -t $session:0.0 -h
   tmux split-window -t $session:0.2 -h
   tmux split-window -t $session:0.4 -h
   #tmux split-window -t $session:0.3 -v
  #tmux split-window -t $session:0.6 -h


      # launch Kimera Vio data 
   tmux send-keys -t $session:0.0 'cd ..' C-m
   tmux send-keys -t $session:0.0 'cd ros2_luca_ws/' C-m
   tmux send-keys -t $session:0.0 'source install/setup.bash' C-m
   tmux send-keys -t $session:0.0 'ros2 launch drone_teleoperation hri_server_launch.py name:=quadrotor odom:=odom' C-m #Lancia anche lo script per mesh hololens in rviz
   

  #  #Launch rviz in another session 
  #   #Launch the 
  read -p "Continuing in 0.5 seconds..." -t 0.5
   tmux send-keys -t $session:0.1 'cd  ..' C-m
   tmux send-keys -t $session:0.1 'cd ros2_luca_ws/' C-m
   tmux send-keys -t $session:0.1 'source install/setup.bash' C-m 
   tmux send-keys -t $session:0.1 'ros2 launch scene_understanding_pkg perception_server_launch.py name:=quadrotor odom:=odom' C-m


   read -p "Continuing in 0.5 seconds..." -t 0.5
   tmux send-keys -t $session:0.2 'cd ..' C-m
   tmux send-keys -t $session:0.2 'cd ros2_luca_ws/' C-m
   tmux send-keys -t $session:0.2 'source install/setup.bash' C-m 
   tmux send-keys -t $session:0.2 'ros2 launch drone_teleoperation RRT_planner_launch.py name:=quadrotor odom:=odom' C-m

   read -p "Continuing in 0.5 seconds..." -t 0.5
   tmux send-keys -t $session:0.3 'cd ..' C-m
   tmux send-keys -t $session:0.3 'cd ros2_luca_ws/' C-m
   tmux send-keys -t $session:0.3 'source install/setup.bash' C-m
   tmux send-keys -t $session:0.3 'ros2 launch arpl_autonomy station_hri.launch.py name:=quadrotor haptic:=false' C-m


  

     #      # launch Kimera Vio data 
   tmux send-keys -t $session:0.4 'cd ..' C-m
   tmux send-keys -t $session:0.4 'cd ros2_luca_ws/' C-m
   tmux send-keys -t $session:0.4 'source install/setup.bash' C-m
   tmux send-keys -t $session:0.4 'cd bag' C-m
   tmux send-keys -t $session:0.4 "$STR" C-m 

   
   tmux send-keys -t $session:0.5 'cd  ..' C-m
   tmux send-keys -t $session:0.5 'cd ros2_luca_ws/' C-m
   tmux send-keys -t $session:0.5 'source install/setup.bash' C-m 
   tmux send-keys -t $session:0.5 'ros2 launch arpl_autonomy single_quadrotor_sim.launch.py' C-m




   tmux attach-session -t $session

   
   
###### LAUNCH SIMULATION + HOLO or WEBXR
elif [[ "$n1" -eq 2 ]]; then
  STR1="ros2 bag play "
  STR2="gamelab_map3" #$rosbag 
  STR3=" --remap /race10/nvblox_node/static_occupancy:=/quadrotor/nvblox_node/static_occupancy" #/from_unity/mocap_frame_in_unity_world_coo:=/from_unity/mocap_frame_in_unity_world_coo_dev_null /unity_to_ros/interactive_marker_position:=/unity_to_ros/interactive_marker_position"
  STR="$STR1$STR2$STR3"
  

   tmux split-window -t $session:0.0 -v -p 33  
   tmux split-window -t $session:0.0 -v -p 33 
   tmux split-window -t $session:0.0 -h
   tmux split-window -t $session:0.2 -h
   tmux split-window -t $session:0.4 -h
   tmux split-window -t $session:0.4 -v
   #tmux split-window -t $session:0.3 -v
  #tmux split-window -t $session:0.6 -h


      # launch Kimera Vio data 
   tmux send-keys -t $session:0.0 'cd ..' C-m
   tmux send-keys -t $session:0.0 'cd ros2_luca_ws/' C-m
   tmux send-keys -t $session:0.0 'source install/setup.bash' C-m
   tmux send-keys -t $session:0.0 'ros2 launch drone_teleoperation hri_server_launch.py name:=quadrotor odom:=odom enable_rviz_marker:=false' C-m #Lancia anche lo script per mesh hololens in rviz
   


  read -p "Continuing in 0.5 seconds..." -t 0.5
   tmux send-keys -t $session:0.1 'cd  ..' C-m
   tmux send-keys -t $session:0.1 'cd ros2_luca_ws/' C-m
   tmux send-keys -t $session:0.1 'source install/setup.bash' C-m 
   tmux send-keys -t $session:0.1 'ros2 launch scene_understanding_pkg perception_server_launch.py name:=quadrotor odom:=odom' C-m


   read -p "Continuing in 0.5 seconds..." -t 0.5
   tmux send-keys -t $session:0.2 'cd ..' C-m
   tmux send-keys -t $session:0.2 'cd ros2_luca_ws/' C-m
   tmux send-keys -t $session:0.2 'source install/setup.bash' C-m 
   tmux send-keys -t $session:0.2 'ros2 launch drone_teleoperation RRT_planner_launch.py name:=quadrotor odom:=odom' C-m

   read -p "Continuing in 0.5 seconds..." -t 0.5
   tmux send-keys -t $session:0.3 'cd ..' C-m
   tmux send-keys -t $session:0.3 'cd ros2_luca_ws/' C-m
   tmux send-keys -t $session:0.3 'source install/setup.bash' C-m
   #tmux send-keys -t $session:0.4 'rosrun rviz rviz -d /home/arpl/nyu_frl_ws/src/rviz/admittance_rviz_for_rosbag.rviz' C-m
   tmux send-keys -t $session:0.3 'ros2 launch arpl_autonomy station_hri.launch.py name:=quadrotor haptic:=false' C-m


  

     #      # launch Kimera Vio data 
   tmux send-keys -t $session:0.4 'cd ..' C-m
   tmux send-keys -t $session:0.4 'cd ros2_luca_ws/' C-m
   tmux send-keys -t $session:0.4 'source install/setup.bash' C-m
   tmux send-keys -t $session:0.4 'cd bag' C-m
   tmux send-keys -t $session:0.4 "$STR" C-m 

   
   tmux send-keys -t $session:0.5 'cd  ..' C-m
   tmux send-keys -t $session:0.5 'cd ros2_luca_ws/' C-m
   tmux send-keys -t $session:0.5 'source install/setup.bash' C-m 
   tmux send-keys -t $session:0.5 'ros2 launch arpl_autonomy single_quadrotor_sim.launch.py' C-m
   
   
   #Launch ROS TCP COnnecto 
   read -p "Continuing in 0.5 seconds..." -t 0.5
   tmux send-keys -t $session:0.6 'cd ..' C-m
   tmux send-keys -t $session:0.6 'cd ros2_luca_ws/' C-m
   tmux send-keys -t $session:0.6 'source install/setup.bash' C-m 
   tmux send-keys -t $session:0.6 'ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:='$ROS_IP' -p ROS_TCP_PORT:=10000' C-m  #IP Laptop required
   
     tmux attach-session -t $session
   


############ #Real with Haptic
elif  [[ "$n1" -eq 3 ]]; then


  STR1="ros2 bag play "
  STR2="gamelab_map3" #$rosbag 
  STR3=" --remap /race10/nvblox_node/static_occupancy:=/quadrotor/nvblox_node/static_occupancy" #/from_unity/mocap_frame_in_unity_world_coo:=/from_unity/mocap_frame_in_unity_world_coo_dev_null /unity_to_ros/interactive_marker_position:=/unity_to_ros/interactive_marker_position"
  STR="$STR1$STR2$STR3"
  

   tmux split-window -t $session:0.0 -v -p 33  
   tmux split-window -t $session:0.0 -v -p 33 
   tmux split-window -t $session:0.0 -h
   tmux split-window -t $session:0.2 -h
   tmux split-window -t $session:0.4 -h
   tmux split-window -t $session:0.4 -v
   #tmux split-window -t $session:0.3 -v
  #tmux split-window -t $session:0.6 -h


      # launch Kimera Vio data 
   tmux send-keys -t $session:0.0 'cd ..' C-m
   tmux send-keys -t $session:0.0 'cd ros2_luca_ws/' C-m
   tmux send-keys -t $session:0.0 'source install/setup.bash' C-m
   tmux send-keys -t $session:0.0 'ros2 launch drone_teleoperation hri_server_launch.py name:=quadrotor odom:=odom' C-m #Lancia anche lo script per mesh hololens in rviz
   

  #  #Launch rviz in another session 
  #   #Launch the 
   tmux send-keys -t $session:0.1 'cd  ..' C-m
   tmux send-keys -t $session:0.1 'cd ros2_luca_ws/' C-m
   tmux send-keys -t $session:0.1 'source install/setup.bash' C-m 
   tmux send-keys -t $session:0.1 'ros2 launch scene_understanding_pkg perception_server_launch.py name:=quadrotor odom:=odom' C-m



   tmux send-keys -t $session:0.2 'cd ..' C-m
   tmux send-keys -t $session:0.2 'cd ros2_luca_ws/' C-m
   tmux send-keys -t $session:0.2 'source install/setup.bash' C-m 
   tmux send-keys -t $session:0.2 'ros2 launch drone_teleoperation RRT_planner_launch.py name:=quadrotor odom:=odom' C-m

   read -p "Continuing in 1.0 seconds..." -t 1.0
   tmux send-keys -t $session:0.3 'cd ..' C-m
   tmux send-keys -t $session:0.3 'cd ros2_luca_ws/' C-m
   tmux send-keys -t $session:0.3 'source install/setup.bash' C-m
   #tmux send-keys -t $session:0.4 'rosrun rviz rviz -d /home/arpl/nyu_frl_ws/src/rviz/admittance_rviz_for_rosbag.rviz' C-m
   tmux send-keys -t $session:0.3 'ros2 launch arpl_autonomy station_hri.launch.py name:=quadrotor haptic:=true' C-m


  

     #      # launch Kimera Vio data 
   tmux send-keys -t $session:0.4 'cd ..' C-m
   tmux send-keys -t $session:0.4 'cd ros2_luca_ws/' C-m
   tmux send-keys -t $session:0.4 'source install/setup.bash' C-m
   tmux send-keys -t $session:0.4 'cd bag' C-m
   tmux send-keys -t $session:0.4 "$STR" C-m 

   
   tmux send-keys -t $session:0.5 'cd  ..' C-m
   tmux send-keys -t $session:0.5 'cd ros2_luca_ws/' C-m
   tmux send-keys -t $session:0.5 'source install/setup.bash' C-m 
   tmux send-keys -t $session:0.5 'ros2 launch arpl_autonomy single_quadrotor_sim.launch.py' C-m
   
   tmux send-keys -t $session:0.6 'cd  ..' C-m
   tmux send-keys -t $session:0.6 'cd ros2_luca_ws/' C-m
   tmux send-keys -t $session:0.6 'source install/setup.bash' C-m 
   tmux send-keys -t $session:0.6 'ros2 launch haptic_device_comm_pkg haptic_comm_launch.py' C-m



   tmux attach-session -t $session


#######################  #Real with Holo or Rviz
elif  [[ "$n1" -eq 4 ]]; then

  
   tmux split-window -t $session:0.0 -v -p 33  
   tmux split-window -t $session:0.0 -v -p 33 
   tmux split-window -t $session:0.0 -h


   #tmux split-window -t $session:0.3 -v
  #tmux split-window -t $session:0.6 -h


      # launch Kimera Vio data 
   tmux send-keys -t $session:0.0 'cd ..' C-m
   tmux send-keys -t $session:0.0 'cd ros2_luca_ws/' C-m
   tmux send-keys -t $session:0.0 'source install/setup.bash' C-m
   tmux send-keys -t $session:0.0 'ros2 launch drone_teleoperation hri_server_launch.py name:=race'$number' odom:=odom enable_rviz_marker:=false' C-m #Lancia anche lo script per mesh hololens in rviz
   

  #  #Launch rviz in another session 
  #   #Launch the 
  read -p "Continuing in 0.5 seconds..." -t 0.5
   tmux send-keys -t $session:0.1 'cd  ..' C-m
   tmux send-keys -t $session:0.1 'cd ros2_luca_ws/' C-m
   tmux send-keys -t $session:0.1 'source install/setup.bash' C-m 
   tmux send-keys -t $session:0.1 'ros2 launch scene_understanding_pkg perception_server_launch.py name:=race'$number' odom:=odom' C-m


   read -p "Continuing in 0.5 seconds..." -t 0.5
   tmux send-keys -t $session:0.2 'cd ..' C-m
   tmux send-keys -t $session:0.2 'cd ros2_luca_ws/' C-m
   tmux send-keys -t $session:0.2 'source install/setup.bash' C-m 
   tmux send-keys -t $session:0.2 'ros2 launch drone_teleoperation RRT_planner_launch.py name:=race'$number' odom:=odom' C-m

   
   #Launch ROS TCP COnnecto 
   read -p "Continuing in 0.5 seconds..." -t 0.5
   tmux send-keys -t $session:0.3 'cd ..' C-m
   tmux send-keys -t $session:0.3 'cd ros2_luca_ws/' C-m
   tmux send-keys -t $session:0.3 'source install/setup.bash' C-m 
   tmux send-keys -t $session:0.3 'ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:='$ROS_IP'  -p ROS_TCP_PORT:=10000' C-m  #IP Laptop required

   tmux attach-session -t $session

else 

tmux split-window -t $session:0.0 -v -p 33  
   tmux split-window -t $session:0.0 -v -p 33 
   tmux split-window -t $session:0.0 -h


   #tmux split-window -t $session:0.3 -v
  #tmux split-window -t $session:0.6 -h


      # launch Kimera Vio data 
   tmux send-keys -t $session:0.0 'cd ..' C-m
   tmux send-keys -t $session:0.0 'cd ros2_luca_ws/' C-m
   tmux send-keys -t $session:0.0 'source install/setup.bash' C-m
   tmux send-keys -t $session:0.0 'ros2 launch drone_teleoperation hri_server_launch.py name:=race'$number' odom:=odom' C-m #Lancia anche lo script per mesh hololens in rviz
   

  #  #Launch rviz in another session 
  #   #Launch the 
  read -p "Continuing in 0.5 seconds..." -t 0.5
   tmux send-keys -t $session:0.1 'cd  ..' C-m
   tmux send-keys -t $session:0.1 'cd ros2_luca_ws/' C-m
   tmux send-keys -t $session:0.1 'source install/setup.bash' C-m 
   tmux send-keys -t $session:0.1 'ros2 launch scene_understanding_pkg perception_server_launch.py name:=race'$number' odom:=odom' C-m


   read -p "Continuing in 0.5 seconds..." -t 0.5
   tmux send-keys -t $session:0.2 'cd ..' C-m
   tmux send-keys -t $session:0.2 'cd ros2_luca_ws/' C-m
   tmux send-keys -t $session:0.2 'source install/setup.bash' C-m 
   tmux send-keys -t $session:0.2 'ros2 launch drone_teleoperation RRT_planner_launch.py name:=race'$number' odom:=odom' C-m

   

     
   tmux send-keys -t $session:0.3 'cd  ..' C-m
   tmux send-keys -t $session:0.3 'cd ros2_luca_ws/' C-m
   tmux send-keys -t $session:0.3 'source install/setup.bash' C-m 
   tmux send-keys -t $session:0.3 'ros2 launch haptic_device_comm_pkg haptic_comm_launch.py' C-m
   
   

   tmux attach-session -t $session
   

  

fi






