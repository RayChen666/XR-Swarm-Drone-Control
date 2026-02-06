import websocket
import ssl
import json
import threading
import numpy as np
import time

class Pose:
    x: float
    y: float
    z: float
    #orientation
    q1: float
    q2: float
    q3: float
    q4: float
    frame_id: str
    time: float

class websocket_connection(object):
    def __init__(self,  channel_, type_):
        #Channel: The topic name to subscribe
    
        #type: Publish or subscribe
        self.channel = channel_
        self.event = {
        "request": type_,  #or Publish depends the type of desired connection
        "message": '',
        "channel": self.channel,
        }
        self.data_rcv = Pose() #Type of message to send as publisher (can be customized in the function )
        self.desired_task = 0
        self.desired_waypoints = []
        self.start_traj_flag = False
        self.take_control = False
        #Odom to publish
        #self.robot_odom = Pose()
        self.desired_pose_message_received = False
        self.desired_task_message_received = False
        self.desired_waypoint_list_message_received = False
        self.start_traj_sig_received = False
        self.start_take_control_received = False

        self.traj_updated = False
        self.path_updated = False
        

    def on_open(self, ws):
        #This function fired up the communication asking to connect as a subscriber
        print('Opened Connection')
        ws.send(json.dumps(self.event))

    

    def on_open_pub(self, ws_):
        #This fucntion ask the cinnection as a publisher 
        print('Opened Publisher Connection')

        ws_.send(json.dumps(self.event))
        time_now= time.time()
        time_loop = time.time()
        delta_time = time_loop - time_now
        time_open_conn = 10000 #second
        while (delta_time < time_open_conn):
            #print("Sending Odom Message")
            for s in self.event:
                if (self.event[s] == 'webxr_traj' and self.traj_updated):
                    print(self.event[s])
                    ws_.send(json.dumps(self.event))
                    self.traj_updated = False
                elif (self.event[s] == 'webxr_robot_pos'):
                    ws_.send(json.dumps(self.event))
                elif (self.event[s] == 'webxr_robot_pos2_1'):
                    ws_.send(json.dumps(self.event))
                elif (self.event[s] == 'webxr_robot_pos2_5'):
                    ws_.send(json.dumps(self.event))
                elif (self.event[s] == 'webxr_robot_pos2_8'):
                    ws_.send(json.dumps(self.event))
                elif (self.event[s] == 'webxr_robot_pos2_9'):
                    ws_.send(json.dumps(self.event))
                elif (self.event[s] == 'webxr_static_occupancy'):
                    ws_.send(json.dumps(self.event))
                elif (self.event[s] == 'webxr_final_goal'):
                    ws_.send(json.dumps(self.event))
                elif (self.event[s] == 'webxr_mpl_path'):
                    ws_.send(json.dumps(self.event))
                    break
           
            time_loop = time.time()
            delta_time = time_loop - time_now
            time.sleep(0.2) # test for frequency
    


    def on_message_des_user_pos(self, ws, message):
        event = json.loads(message) 
        print("event: ", event)
        #self.data_rcv = event
        for s in event:
            if (s == 'position_msg'):
                self.data_rcv.x = event[s][0]
                self.data_rcv.y = event[s][1]
                self.data_rcv.z = event[s][2]
            elif (s == 'orientation_msg'):
                self.data_rcv.q1 = event[s][0]
                self.data_rcv.q2 = event[s][1]
                self.data_rcv.q3 = event[s][2]
                self.data_rcv.q4 = event[s][3]
            elif (s == 'frame_id'):
                #Parse the message since it is a string
                frame_id = ''
                for ii in range(len(event[s])):
                    frame_id  = frame_id + event[s][ii]
                self.data_rcv.frame_id =  frame_id
        self.desired_pose_message_received = True
            #split_string = web_sub.data_rcv.split("[")
    
    def on_message_des_task(self, ws, message):
        event = json.loads(message) 
        print("event: ", event)
        for s in event:
            if (s == 'task'):
                self.desired_task= event[s]
            elif (s == 'frame_id'):
                #Parse the message since it is a string
                frame_id = ''
                for ii in range(len(event[s])):
                    frame_id  = frame_id + event[s][ii]
                self.data_rcv.frame_id =  frame_id
        print("self.desired_task: ", self.desired_task)
        self.desired_task_message_received = True
    
    def on_message_des_waypoints(self, ws, message):
        event = json.loads(message) 
        print("event: ", event)
        for s in event:
            if (s == 'position_list_msg'):
                self.desired_waypoints = event[s]
            elif (s == 'frame_id'):
                #Parse the message since it is a string
                frame_id = ''
                for ii in range(len(event[s])):
                    frame_id  = frame_id + event[s][ii]
                self.data_rcv.frame_id =  frame_id

        self.desired_waypoint_list_message_received = True

    def on_message_start_traj(self, ws, message):
        event = json.loads(message) 
        print("event: ", event)
        for s in event:
            if (s == 'flag'):
                self.start_traj_flag = True
        self.start_traj_sig_received = True

    def on_message_take_control(self, ws, message):
        event = json.loads(message) 
        print("Take Control event: ", event)
        for s in event:
            if (s == 'flag'):
                self.take_control = True
        self.start_take_control_received = True

    def on_close(self, ws):
        print('Closed Connection')
        ws.close()

    def on_error(self, ws, err):
       print("Got a an error: ", err)