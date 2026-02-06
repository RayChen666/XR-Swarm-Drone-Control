#!/bin/bash

session="NYU-FRL"

tmux new-session -d -s $session -n home

tmux split-window -t $session:0.0 -h -p 33  
tmux split-window -t $session:0.0 -h -p 33 




      # launch NYU-FRL-XR server
tmux send-keys -t $session:0.0 'cd ..' C-m
tmux send-keys -t $session:0.0 'cd ros2_luca_ws/src/' C-m
tmux send-keys -t $session:0.0 'cd NYU-FRL-XR/' C-m
sleep 2.0
tmux send-keys -t $session:0.0 './startserver' C-m 
sleep 2.0
tmux send-keys -t $session:0.0 './startserver' C-m 



#  # LAunch the DDS framework for websocket 
#read -p "Please go to the Webxr Google chrome page and press the button enter to continue with launching sequence ..."


sleep 2.0
tmux send-keys -t $session:0.1 'cd  ..' C-m
tmux send-keys -t $session:0.1 'cd ros2_luca_ws/src/' C-m
tmux send-keys -t $session:0.1 'cd NYU-FRL-XR/js/ARPL-test/pubsub-master/' C-m 
tmux send-keys -t $session:0.1 'node index.js' C-m

 # LAunch the node script for websocket ros2 communication
sleep 2.0
tmux send-keys -t $session:0.2 'cd ..' C-m
tmux send-keys -t $session:0.2 'cd ros2_luca_ws/' C-m
tmux send-keys -t $session:0.2 'source install/setup.bash' C-m 
tmux send-keys -t $session:0.2 'cd src/websocket_connection/src/' C-m
tmux send-keys -t $session:0.2 'python3 client.py' C-m

tmux attach-session -t $session

  


