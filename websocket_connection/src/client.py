#!/usr/bin/env python3
# from build.std_msgs.rosidl_generator_py.std_msgs import msg
import rclpy
from rclpy.node import Node
import websocket
import ssl
import json
import threading
import numpy as np
import time
from dataclasses import dataclass
from websocket_connection import websocket_connection as webc
from websocket_connection import Pose as Pose  # Assuming this is defined
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from nav_msgs.msg import Path
from std_msgs.msg import Int32
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
import sensor_msgs_py.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2

# This should be changed according to local ip address
host = "0.0.0.0"
port = 8080
class ROS_Wrapper(Node):

    def __init__(self):

        super().__init__('VR_ROS_wrapper')

        self.pub_des_pose = self.create_publisher(PoseStamped, '/from_NYU_FRL/des_pose', 10)
        self.get_logger().info("Initialized des pose publisher.")

        self.pub_des_task_n = self.create_publisher(Int32, '/from_NYU_FRL/des_task_number', 10)
        self.get_logger().info("Initialized task number publisher.")

        self.pub_des_waypoints = self.create_publisher(Path, '/from_NYU_FRL/des_waypoints', 10)
        self.get_logger().info("Initialized des waypoints publisher.")

        self.pub_start_traj_sig = self.create_publisher(Bool, '/from_NYU_FRL/start_des_traj_sign', 10)
        self.get_logger().info("Initialized Traj Start Signal publisher.")

        self.pub_take_control = self.create_publisher(Bool, '/from_NYU_FRL/take_control', 10)
        self.get_logger().info("Initialized Take Control Signal publisher.")

        #self.create_subscription(Odometry, '/voxl/odom', self.odometry_callback, 10)
        self.create_subscription(Odometry, '/race10/odom', self.odometry_callback, 10)
        self.create_subscription(Odometry, '/quadrotor/odom', self.odometry_callback, 10) # if test with single drone put quadrotor here
        # /voxl2_1  /voxl2_5  /voxl2_8  /voxl2_9 /voxl2_0
        self.create_subscription(Odometry, '/voxl2_1/odom', self.odometry_callback2_1, 10)
        self.create_subscription(Odometry, '/voxl2_5/odom', self.odometry_callback2_5, 10) 
        self.create_subscription(Odometry, '/voxl2_8/odom', self.odometry_callback2_8, 10)
        self.create_subscription(Odometry, '/voxl2_9/odom', self.odometry_callback2_9, 10)


        self.get_logger().info("Initialized the odom subscriber.")
        
        

        self.create_subscription(Path, '/to_webxr/qp_trajectory_pos', self.path_callback, 10)
        self.get_logger().info("Initialized the pos path subscriber.")

        self.create_subscription(Path, '/to_webxr/qp_trajectory_pos', self.path_vel_callback, 10)
        self.get_logger().info("Initialized the vel path subscriber.")

        # add kind of subscriber
        self.create_subscription(PointCloud2, '/race10/nvblox_node/static_occupancy', self.static_occupancy_callback, 10)
        # self.create_subscription(PointCloud2, '/multi_robots/nvblox_node/static_occupancy', self.static_occupancy_callback, 10)
        self.get_logger().info("Initialized map subscriber.")

        # add expected goal subscriber
        self.create_subscription(Point, '/rrt/final_goal', self.final_goal_callback, 10)
        self.get_logger().info("Initialized goal subscriber.")

        self.create_subscription(Path, '/multi_robots/mpl_path', self.mpl_path_callback, 10)
        self.get_logger().info("Initialized mpl path subscriber.")
        
         # Retrieve host and port from parameters

        # Check if parameters exist before declaring
        if not self.has_parameter('host'):
            self.declare_parameter('host', '192.168.1.188')  # Declare with default host
            self.get_logger().info("Parameter 'host' declared with default.")
        else:
            self.get_logger().info("Parameter 'host' already exists.")

        if not self.has_parameter('port'):
            self.declare_parameter('port', 8080)             # Declare with default port
            self.get_logger().info("Parameter 'port' declared with default.")
        else:
            self.get_logger().info("Parameter 'port' already exists.")

        # Retrieve parameter values (now they are guaranteed to exist)
        host = self.get_parameter('host').get_parameter_value().string_value
        port = self.get_parameter('port').get_parameter_value().integer_value
        

        self.robot_pose = Odometry()
        self.robot_pose2_1 = Odometry()
        self.robot_pose2_5 = Odometry()
        self.robot_pose2_8 = Odometry()
        self.robot_pose2_9 = Odometry()
        

        self.desired_traj_pos = []
        self.desired_traj_vel = []
        self.sampling_val = 10
        self.traj_send = False
        
        self.static_occupancy = PointCloud2()
        self.final_goal = Point()
        self.static_occupancy_points = []
        self.occupancy_send = False
        self.sampling_val_occupancy = 1 
        # Sample every 5th point, you can change this to avoid lagging issue 
        # self.sampling_val_occupancy = 5

        self.desired_path_pos = []


    # Define the publisher from FRL to ROS of the user desired pose
    def publish_des_pose_msg(self, message):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg() # Use rclpy's clock
        msg.header.frame_id = message.data_rcv.frame_id
        msg.pose.position.x = float(message.data_rcv.x)
        msg.pose.position.y = -1 * float(message.data_rcv.z)
        msg.pose.position.z = float(message.data_rcv.y)
        msg.pose.orientation.x = float(message.data_rcv.q1)
        msg.pose.orientation.y = float(message.data_rcv.q2)
        msg.pose.orientation.z = float(message.data_rcv.q3)
        msg.pose.orientation.w = float(message.data_rcv.q4)
        self.pub_des_pose.publish(msg)

      # Republish the desired task if received, to organize the switch of the code
    def publish_des_task_number(self, message):
        msg = Int32()
        msg.data = message.desired_task
        print("Task Number Received: ", msg.data)
        self.pub_des_task_n.publish(msg)

    def publish_des_waypoints(self, message):
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()  # Use ROS2 time
        # msg.header.frame_id = message.data_rcv.frame_id  # Assuming data_rcv exists

        for i in range(len(message.desired_waypoints)):
            msg_pose_stamp = PoseStamped()
            msg_pose_stamp.header.stamp = self.get_clock().now().to_msg()  # Use ROS2 time
            msg_pose_stamp.header.frame_id = message.data_rcv.frame_id # Assuming data_rcv exists
            msg_pose_stamp.pose.position.x = message.desired_waypoints[i][0]
            msg_pose_stamp.pose.position.y = -1 * message.desired_waypoints[i][2]
            msg_pose_stamp.pose.position.z = message.desired_waypoints[i][1]
            msg.poses.append(msg_pose_stamp)
            
            print("WAYPOINTS: x: {}, y: {}, z: {}".format(message.desired_waypoints[i][0],  -1*message.desired_waypoints[i][2],message.desired_waypoints[i][1]) )
        self.pub_des_waypoints.publish(msg)
        self.traj_send = False

    
    def publish_start_traj_signal(self, message):
        msg = Bool()
        msg.data = True
        self.pub_start_traj_sig.publish(msg)

    def publish_take_control_signal(self, message):
        msg = Bool()
        msg.data = True
        self.pub_take_control.publish(msg)

    # Define the drone odom sub to submit to the FRL application

    # odom callback for drone quadrotor
    def odometry_callback(self, msg):                                   
        # self.robot_pose.header.frame_id = msg.header.frame_id
        self.robot_pose.pose.pose.position = msg.pose.pose.position
        self.robot_pose.pose.pose.orientation = msg.pose.pose.orientation
        self.robot_pose.twist.twist.linear = msg.twist.twist.linear

    # odom callback for drone voxl2_1
    def odometry_callback2_1 (self, msg):
        self.robot_pose2_1.pose.pose.position = msg.pose.pose.position
        self.robot_pose2_1.pose.pose.orientation = msg.pose.pose.orientation
        self.robot_pose2_1.twist.twist.linear = msg.twist.twist.linear
    # odom callback for drone voxl2_5
    def odometry_callback2_5 (self, msg):
        self.robot_pose2_5.pose.pose.position = msg.pose.pose.position
        self.robot_pose2_5.pose.pose.orientation = msg.pose.pose.orientation
        self.robot_pose2_5.twist.twist.linear = msg.twist.twist.linear

    # odom callback for drone voxl2_8
    def odometry_callback2_8 (self, msg):
        self.robot_pose2_8.pose.pose.position = msg.pose.pose.position
        self.robot_pose2_8.pose.pose.orientation = msg.pose.pose.orientation
        self.robot_pose2_8.twist.twist.linear = msg.twist.twist.linear

    # odom callback for drone voxl2_9
    def odometry_callback2_9 (self, msg):
        self.robot_pose2_9.pose.pose.position = msg.pose.pose.position
        self.robot_pose2_9.pose.pose.orientation = msg.pose.pose.orientation
        self.robot_pose2_9.twist.twist.linear = msg.twist.twist.linear


    def path_callback(self, msg):
        if self.traj_send:
            return
        self.desired_traj_pos = []
        self.sampling_val = 10
        sampled_len = round(len(msg.poses) / self.sampling_val)
        counter = 0
        print("Len Msg Poses: ", len(msg.poses))
        if (len(msg.poses) > 0):
            for i in range(len(msg.poses)):
                x = msg.poses[counter].pose.position.x
                y = msg.poses[counter].pose.position.y
                z = msg.poses[counter].pose.position.z
                pose = np.array([x, y, z])
                self.desired_traj_pos.append(pose)
                counter += self.sampling_val

                if counter > len(msg.poses):

                    break
    
    def path_vel_callback(self, msg):
        if self.traj_send:
            return
        self.desired_traj_vel = []

        # change this to 1 for the currrent publish command
        self.sampling_val = 10
        sampled_len = round(len(msg.poses) / self.sampling_val)
        counter = 0
        if (len(msg.poses) > 0):
            for i in range(len(msg.poses)):
                x = msg.poses[counter].pose.position.x
                y = msg.poses[counter].pose.position.y
                z = msg.poses[counter].pose.position.z
                vel = np.array([x, y, z])
                self.desired_traj_vel.append(vel)
                counter += self.sampling_val
                if counter > len(msg.poses):
                    break

    def static_occupancy_callback(self, msg):

        if self.occupancy_send:  
            return
        
        self.static_occupancy = msg  
        self.static_occupancy_points = [] 
        
        # Parse and sample data (like path_callback does)
        points_generator = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        counter = 0
        for point in points_generator:
            if counter % self.sampling_val_occupancy == 0: 
                ros_x, ros_y, ros_z = point
            
                # Use SAME transformation as robot
                webxr_x = float(ros_x)      # ROS X → WebXR X
                webxr_y = float(ros_z)      # ROS Z → WebXR Y
                webxr_z = float(-ros_y)     # ROS Y → WebXR -Z

                if webxr_y > 0.11:
                    point_array = np.array([webxr_x, webxr_y, webxr_z])
                    self.static_occupancy_points.append(point_array)

            counter += 1
            if len(self.static_occupancy_points) >= 5000:  # Limit total points
                break

    def final_goal_callback(self, msg):
        self.final_goal = msg


    def mpl_path_callback(self, msg):
        self.desired_path_pos = []
        # self.sampling_val = 10
        self.sampling_val = 1
        sampled_len = round(len(msg.poses) / self.sampling_val)
        counter = 0
        print("Len Msg Poses: ", len(msg.poses))
        if (len(msg.poses) > 0):
            print("path received")
            for i in range(len(msg.poses)):
                x = msg.poses[counter].pose.position.x
                y = msg.poses[counter].pose.position.y
                z = msg.poses[counter].pose.position.z
                pose = np.array([x, y, z])
                self.desired_path_pos.append(pose)
                counter += self.sampling_val

                if counter > len(msg.poses):

                    break
    



''' END of the class '''



def subscriber_threading(socket, web_sub, ros_wr):
    ws = websocket.WebSocketApp(socket, on_open=web_sub.on_open, on_message=web_sub.on_message_des_user_pos, on_error=web_sub.on_error)
    ws.run_forever(sslopt={"cert_reqs": ssl.CERT_NONE})

def subscriber_task_threading(socket, web_sub, ros_wr):
    ws = websocket.WebSocketApp(socket, on_open=web_sub.on_open, on_message=web_sub.on_message_des_task, on_error=web_sub.on_error)
    ws.run_forever(sslopt={"cert_reqs": ssl.CERT_NONE})

def subscriber_waypoints_threading(socket, web_sub, ros_wr):
    ws = websocket.WebSocketApp(socket, on_open=web_sub.on_open, on_message=web_sub.on_message_des_waypoints, on_error=web_sub.on_error)
    ws.run_forever(sslopt={"cert_reqs": ssl.CERT_NONE})


def subscriber_start_traj_threading(socket, web_sub, ros_wr):
    ws = websocket.WebSocketApp(socket, on_open=web_sub.on_open, on_message=web_sub.on_message_start_traj, on_error=web_sub.on_error)
    ws.run_forever(sslopt={"cert_reqs": ssl.CERT_NONE})

def subscriber_take_control_threading(socket, web_sub, ros_wr):
    ws = websocket.WebSocketApp(socket, on_open=web_sub.on_open, on_message=web_sub.on_message_take_control, on_error=web_sub.on_error)
    ws.run_forever(sslopt={"cert_reqs": ssl.CERT_NONE})

def publisher_to_FRL_app_threading(socket, web_pub, ros_wr): # publisher for odometry
    x = ros_wr.robot_pose.pose.pose.position.x
    y = ros_wr.robot_pose.pose.pose.position.y
    z = ros_wr.robot_pose.pose.pose.position.z
    q1 = ros_wr.robot_pose.pose.pose.orientation.x
    q2 = ros_wr.robot_pose.pose.pose.orientation.y
    q3 = ros_wr.robot_pose.pose.pose.orientation.z
    qw = ros_wr.robot_pose.pose.pose.orientation.w
    vx = ros_wr.robot_pose.twist.twist.linear.x
    vy = ros_wr.robot_pose.twist.twist.linear.y
    vz = ros_wr.robot_pose.twist.twist.linear.z

    position_msg = [x, y, z]
    orientation_msg = [q1, q2, q3, qw]
    velocity_msg = [vx, vy, vz]
    message = {"ts": time.time(), "frame_id": "world", "name":"quadrotor", "position_msg": position_msg, "orientation_msg": orientation_msg, "velocity_msg": velocity_msg}

    web_pub.event = {
        "request": "PUBLISH",
        "type": 'webxr_robot_pos',
        "message": message,
        "channel": web_pub.channel
    }
    # print("Message Published to FRL: ", position_msg)
    ws = websocket.WebSocketApp(socket, on_open=web_pub.on_open_pub)
    ws.run_forever(sslopt={"cert_reqs": ssl.CERT_NONE})

# publisher of odom of drone voxl2_1
def publisher_to_FRL_app_threading2_1 (socket, web_pub, ros_wr):
    x = ros_wr.robot_pose2_1.pose.pose.position.x
    y = ros_wr.robot_pose2_1.pose.pose.position.y
    z = ros_wr.robot_pose2_1.pose.pose.position.z
    q1 = ros_wr.robot_pose2_1.pose.pose.orientation.x
    q2 = ros_wr.robot_pose2_1.pose.pose.orientation.y
    q3 = ros_wr.robot_pose2_1.pose.pose.orientation.z
    qw = ros_wr.robot_pose2_1.pose.pose.orientation.w
    vx = ros_wr.robot_pose2_1.twist.twist.linear.x
    vy = ros_wr.robot_pose2_1.twist.twist.linear.y
    vz = ros_wr.robot_pose2_1.twist.twist.linear.z

    position_msg = [x, y, z]
    orientation_msg = [q1, q2, q3, qw]
    velocity_msg = [vx, vy, vz]

    message = {"ts": time.time(), "frame_id": "world", "name":"voxl2_1", "position_msg": position_msg, "orientation_msg": orientation_msg, "velocity_msg": velocity_msg}
    web_pub.event = {
        "request": "PUBLISH",
        "type": 'webxr_robot_pos2_1',
        "message": message,
        "channel": web_pub.channel
    }

    ws = websocket.WebSocketApp(socket, on_open=web_pub.on_open_pub)
    ws.run_forever(sslopt={"cert_reqs": ssl.CERT_NONE})

# publisher of odom of drone voxl2_5
def publisher_to_FRL_app_threading2_5 (socket, web_pub, ros_wr):
    x = ros_wr.robot_pose2_5.pose.pose.position.x
    y = ros_wr.robot_pose2_5.pose.pose.position.y
    z = ros_wr.robot_pose2_5.pose.pose.position.z
    q1 = ros_wr.robot_pose2_5.pose.pose.orientation.x
    q2 = ros_wr.robot_pose2_5.pose.pose.orientation.y
    q3 = ros_wr.robot_pose2_5.pose.pose.orientation.z
    qw = ros_wr.robot_pose2_5.pose.pose.orientation.w
    vx = ros_wr.robot_pose2_5.twist.twist.linear.x
    vy = ros_wr.robot_pose2_5.twist.twist.linear.y
    vz = ros_wr.robot_pose2_5.twist.twist.linear.z

    position_msg = [x, y, z]
    orientation_msg = [q1, q2, q3, qw]
    velocity_msg = [vx, vy, vz]

    message = {"ts": time.time(), "frame_id": "world", "name":"voxl2_5", "position_msg": position_msg, "orientation_msg": orientation_msg, "velocity_msg": velocity_msg}
    web_pub.event = {
        "request": "PUBLISH",
        "type": 'webxr_robot_pos2_5',
        "message": message,
        "channel": web_pub.channel
    }

    ws = websocket.WebSocketApp(socket, on_open=web_pub.on_open_pub)
    ws.run_forever(sslopt={"cert_reqs": ssl.CERT_NONE})

# publisher of odom of drone voxl2_8
def publisher_to_FRL_app_threading2_8 (socket, web_pub, ros_wr):
    x = ros_wr.robot_pose2_8.pose.pose.position.x
    y = ros_wr.robot_pose2_8.pose.pose.position.y
    z = ros_wr.robot_pose2_8.pose.pose.position.z
    q1 = ros_wr.robot_pose2_8.pose.pose.orientation.x
    q2 = ros_wr.robot_pose2_8.pose.pose.orientation.y
    q3 = ros_wr.robot_pose2_8.pose.pose.orientation.z
    qw = ros_wr.robot_pose2_8.pose.pose.orientation.w
    vx = ros_wr.robot_pose2_8.twist.twist.linear.x
    vy = ros_wr.robot_pose2_8.twist.twist.linear.y
    vz = ros_wr.robot_pose2_8.twist.twist.linear.z

    position_msg = [x, y, z]
    orientation_msg = [q1, q2, q3, qw]
    velocity_msg = [vx, vy, vz]

    message = {"ts": time.time(), "frame_id": "world", "name":"voxl2_8", "position_msg": position_msg, "orientation_msg": orientation_msg, "velocity_msg": velocity_msg}
    web_pub.event = {
        "request": "PUBLISH",
        "type": 'webxr_robot_pos2_8',
        "message": message,
        "channel": web_pub.channel
    }

    ws = websocket.WebSocketApp(socket, on_open=web_pub.on_open_pub)
    ws.run_forever(sslopt={"cert_reqs": ssl.CERT_NONE})

# publisher of odom of drone voxl2_9
def publisher_to_FRL_app_threading2_9 (socket, web_pub, ros_wr):
    x = ros_wr.robot_pose2_9.pose.pose.position.x
    y = ros_wr.robot_pose2_9.pose.pose.position.y
    z = ros_wr.robot_pose2_9.pose.pose.position.z
    q1 = ros_wr.robot_pose2_9.pose.pose.orientation.x
    q2 = ros_wr.robot_pose2_9.pose.pose.orientation.y
    q3 = ros_wr.robot_pose2_9.pose.pose.orientation.z
    qw = ros_wr.robot_pose2_9.pose.pose.orientation.w
    vx = ros_wr.robot_pose2_9.twist.twist.linear.x
    vy = ros_wr.robot_pose2_9.twist.twist.linear.y
    vz = ros_wr.robot_pose2_9.twist.twist.linear.z

    position_msg = [x, y, z]
    orientation_msg = [q1, q2, q3, qw]
    velocity_msg = [vx, vy, vz]

    message = {"ts": time.time(), "frame_id": "world", "name":"voxl2_9", "position_msg": position_msg, "orientation_msg": orientation_msg, "velocity_msg": velocity_msg}
    web_pub.event = {
        "request": "PUBLISH",
        "type": 'webxr_robot_pos2_9',
        "message": message,
        "channel": web_pub.channel
    }

    ws = websocket.WebSocketApp(socket, on_open=web_pub.on_open_pub)
    ws.run_forever(sslopt={"cert_reqs": ssl.CERT_NONE})


def publisher_to_FRL_app_robot_path(socket, web_pub, ros_wr):
    if ros_wr.traj_send:
        return

    path_msg = ros_wr.desired_traj_pos
    velocity_msg = ros_wr.desired_traj_vel
    acceleration_msg = []
    message = {"ts": time.time(), "frame_id": "world", "des_position_msg": path_msg, "des_velocity_msg": velocity_msg, "des_acceleration_msg": acceleration_msg}
    web_pub.event = {
        "request": "PUBLISH",
        "type": 'webxr_robot_traj',
        "message": message,
        "channel": web_pub.channel
    }
    ros_wr.traj_send = True
    
    ws = websocket.WebSocketApp(socket, on_open=web_pub.on_open_pub)
    ws.run_forever(sslopt={"cert_reqs": ssl.CERT_NONE})

def publisher_to_FRL_app_static_occupancy(socket, web_pub, ros_wr):
    while rclpy.ok():
        if len(ros_wr.static_occupancy_points) == 0:
            time.sleep(0.2)
            continue

        points = [point.tolist() for point in ros_wr.static_occupancy_points]
        message = {"ts": time.time(), "frame_id": "world", "points": points}
        
        web_pub.event = {
            "request": "PUBLISH",
            "type": 'webxr_static_occupancy',
            "message": message,
            "channel": web_pub.channel
        }
        # print("✅ Map Published to FRL:", message)
        ws = websocket.WebSocketApp(socket, on_open=web_pub.on_open_pub)
        ws.run_forever(sslopt={"cert_reqs": ssl.CERT_NONE})
        break

def publisher_to_FRL_app_final_goal(socket, web_pub, ros_wr):
    x = ros_wr.final_goal.x
    y = ros_wr.final_goal.y
    z = ros_wr.final_goal.z

    goal_location_msg = [x, y, z]
   
    message = {"ts": time.time(), "frame_id": "world", "goal_location": goal_location_msg}
    web_pub.event = {
        "request": "PUBLISH",
        "type": 'webxr_final_goal',
        "message": message,
        "channel": web_pub.channel
    }
    # print("Message Published to FRL: ", goal_location_msg)
    ws = websocket.WebSocketApp(socket, on_open=web_pub.on_open_pub)
    ws.run_forever(sslopt={"cert_reqs": ssl.CERT_NONE})

def publisher_to_FRL_app_mpl_path(socket, web_pub, ros_wr):
    path_msg = ros_wr.desired_path_pos
    message = {"ts": time.time(), "frame_id": "world", "des_position_msg": path_msg}
    web_pub.event = {
        "request": "PUBLISH",
        "type": 'webxr_mpl_path',
        "message": message,
        "channel": web_pub.channel
    }
    
    ws = websocket.WebSocketApp(socket, on_open=web_pub.on_open_pub)
    ws.run_forever(sslopt={"cert_reqs": ssl.CERT_NONE})

# update odom for drone quadrotor
def update_robot_odom_to_FRL_app(socket, ros_wr, web_pub):
    #This method is used to update the pose with the latest odom received 
    x = ros_wr.robot_pose.pose.pose.position.x
    y = ros_wr.robot_pose.pose.pose.position.z
    z = -1*ros_wr.robot_pose.pose.pose.position.y
    q1 = ros_wr.robot_pose.pose.pose.orientation.x
    q2 = ros_wr.robot_pose.pose.pose.orientation.y
    q3 = ros_wr.robot_pose.pose.pose.orientation.z
    qw = ros_wr.robot_pose.pose.pose.orientation.w
    vx = ros_wr.robot_pose.twist.twist.linear.x
    vy = ros_wr.robot_pose.twist.twist.linear.y
    vz = ros_wr.robot_pose.twist.twist.linear.z
    
    position_msg = [x, y, z]
    orientation_msg = [q1, q2, q3, qw] #q1,q2,q3,q4
    velocity_msg = [vx, vy, vz]

    # print("quadrotor: ", position_msg)
    message = {"ts": time.time(), "frame_id": "world", "name":"quadrotor", "position_msg": position_msg, "orientation_msg": orientation_msg, "velocity_msg": velocity_msg}
    web_pub.event = {
            "request": "PUBLISH",
            "type": 'webxr_robot_pos',
            "message": message,
            "channel": web_pub.channel
    }

# update odom for drone voxl2_1
def update_robot_odom_to_FRL_app2_1(socket, ros_wr, web_pub):
    x = ros_wr.robot_pose2_1.pose.pose.position.x
    y = ros_wr.robot_pose2_1.pose.pose.position.z
    z = -1*ros_wr.robot_pose2_1.pose.pose.position.y
    q1 = ros_wr.robot_pose2_1.pose.pose.orientation.x
    q2 = ros_wr.robot_pose2_1.pose.pose.orientation.y
    q3 = ros_wr.robot_pose2_1.pose.pose.orientation.z
    qw = ros_wr.robot_pose2_1.pose.pose.orientation.w
    vx = ros_wr.robot_pose2_1.twist.twist.linear.x
    vy = ros_wr.robot_pose2_1.twist.twist.linear.y
    vz = ros_wr.robot_pose2_1.twist.twist.linear.z
    
    position_msg = [x, y, z]
    orientation_msg = [q1, q2, q3, qw] #q1,q2,q3,q4
    velocity_msg = [vx, vy, vz]

    # print("voxl2_1: ", position_msg)
    message = {"ts": time.time(), "frame_id": "world", "name":"voxl2_1", "position_msg": position_msg, "orientation_msg": orientation_msg, "velocity_msg": velocity_msg}
    web_pub.event = {
            "request": "PUBLISH",
            "type": 'webxr_robot_pos2_1',
            "message": message,
            "channel": web_pub.channel
    }

# update odom for drone volx2_5
def update_robot_odom_to_FRL_app2_5(socket, ros_wr, web_pub):
    x = ros_wr.robot_pose2_5.pose.pose.position.x
    y = ros_wr.robot_pose2_5.pose.pose.position.z
    z = -1*ros_wr.robot_pose2_5.pose.pose.position.y
    q1 = ros_wr.robot_pose2_5.pose.pose.orientation.x
    q2 = ros_wr.robot_pose2_5.pose.pose.orientation.y
    q3 = ros_wr.robot_pose2_5.pose.pose.orientation.z
    qw = ros_wr.robot_pose2_5.pose.pose.orientation.w
    vx = ros_wr.robot_pose2_5.twist.twist.linear.x
    vy = ros_wr.robot_pose2_5.twist.twist.linear.y
    vz = ros_wr.robot_pose2_5.twist.twist.linear.z
    
    position_msg = [x, y, z]
    orientation_msg = [q1, q2, q3, qw] #q1,q2,q3,q4
    velocity_msg = [vx, vy, vz]

    # print("voxl2_5: ", position_msg)
    message = {"ts": time.time(), "frame_id": "world", "name":"voxl2_5", "position_msg": position_msg, "orientation_msg": orientation_msg, "velocity_msg": velocity_msg}
    web_pub.event = {
            "request": "PUBLISH",
            "type": 'webxr_robot_pos2_5',
            "message": message,
            "channel": web_pub.channel
    }

# update odom for drone volx2_8
def update_robot_odom_to_FRL_app2_8(socket, ros_wr, web_pub):
    x = ros_wr.robot_pose2_8.pose.pose.position.x
    y = ros_wr.robot_pose2_8.pose.pose.position.z
    z = -1*ros_wr.robot_pose2_8.pose.pose.position.y
    q1 = ros_wr.robot_pose2_8.pose.pose.orientation.x
    q2 = ros_wr.robot_pose2_8.pose.pose.orientation.y
    q3 = ros_wr.robot_pose2_8.pose.pose.orientation.z
    qw = ros_wr.robot_pose2_8.pose.pose.orientation.w
    vx = ros_wr.robot_pose2_8.twist.twist.linear.x
    vy = ros_wr.robot_pose2_8.twist.twist.linear.y
    vz = ros_wr.robot_pose2_8.twist.twist.linear.z
    
    position_msg = [x, y, z]
    orientation_msg = [q1, q2, q3, qw] #q1,q2,q3,q4
    velocity_msg = [vx, vy, vz]

    # print("voxl2_8: ", position_msg)
    message = {"ts": time.time(), "frame_id": "world", "name":"voxl2_8", "position_msg": position_msg, "orientation_msg": orientation_msg, "velocity_msg": velocity_msg}
    web_pub.event = {
            "request": "PUBLISH",
            "type": 'webxr_robot_pos2_8',
            "message": message,
            "channel": web_pub.channel
    }

# update odom for drone volx2_9
def update_robot_odom_to_FRL_app2_9(socket, ros_wr, web_pub):
    x = ros_wr.robot_pose2_9.pose.pose.position.x
    y = ros_wr.robot_pose2_9.pose.pose.position.z
    z = -1*ros_wr.robot_pose2_9.pose.pose.position.y
    q1 = ros_wr.robot_pose2_9.pose.pose.orientation.x
    q2 = ros_wr.robot_pose2_9.pose.pose.orientation.y
    q3 = ros_wr.robot_pose2_9.pose.pose.orientation.z
    qw = ros_wr.robot_pose2_9.pose.pose.orientation.w
    vx = ros_wr.robot_pose2_9.twist.twist.linear.x
    vy = ros_wr.robot_pose2_9.twist.twist.linear.y
    vz = ros_wr.robot_pose2_9.twist.twist.linear.z
    
    position_msg = [x, y, z]
    orientation_msg = [q1, q2, q3, qw] #q1,q2,q3,q4
    velocity_msg = [vx, vy, vz]

    # print("voxl2_9: ", position_msg)
    message = {"ts": time.time(), "frame_id": "world", "name":"voxl2_9", "position_msg": position_msg, "orientation_msg": orientation_msg, "velocity_msg": velocity_msg}
    web_pub.event = {
            "request": "PUBLISH",
            "type": 'webxr_robot_pos2_9',
            "message": message,
            "channel": web_pub.channel
    }


def update_static_occupancy_to_FRL_app(socket, ros_wr, web_pub):
    # type should be 'webxr_static_occupancy'
    # data points for pointCloud2, refer to the documentation of the ROS2 Humble
    if len(ros_wr.static_occupancy_points) == 0:  # Check if data available
        return
    
    # Convert numpy arrays to lists (like update_des_traj_to_FRL_app)
    points = [point.tolist() for point in ros_wr.static_occupancy_points]
    

    message = {"ts": time.time(), "frame_id": "world", "points": points}
    
    web_pub.event = {
        "request": "PUBLISH",
        "type": 'webxr_static_occupancy',
        "message": message,
        "channel": web_pub.channel
    }
    

def update_final_goal_to_FRL_app(socket, ros_wr, web_pub):

    x = ros_wr.final_goal.x
    y = ros_wr.final_goal.z
    z = -ros_wr.final_goal.y
    
    
    goal_location_msg = [x, y, z]
    
    message = {"ts": time.time(), "frame_id": "world", "goal_location": goal_location_msg}
    web_pub.event = {
            "request": "PUBLISH",
            "type": 'webxr_final_goal',
            "message": message,
            "channel": web_pub.channel
    }

def update_mpl_path_to_FRL_app(socket, ros_wr, web_pub):
    # Don't send empty arrays
    if len(ros_wr.desired_path_pos) == 0:
        return  
    

    path_msg = json.dumps([pose.tolist() for pose in ros_wr.desired_path_pos])
    
    message = {"ts": time.time(), "frame_id": "world", "des_position_msg": path_msg}
    web_pub.event = {
            "request": "PUBLISH",
            "type": 'webxr_mpl_path',
            "message": message,
            "channel": web_pub.channel

    }
    print("MPL Path Updated to FRL: n_pos_sample: ",len(ros_wr.desired_path_pos))
    ros_wr.desired_path_pos = []

def update_des_traj_to_FRL_app(ros_wr, web_pub):
    if (len(ros_wr.desired_traj_pos) > 0 and  ros_wr.traj_send == False):
        ros_wr.traj_send = True #This to avoid to send multiple time the same trajectory and blocking the network -> goes to false when new waypoints are receieved 
    else:
        return

    #This method is used to update the pose with the latest odom received 
    path_msg = json.dumps([pose.tolist() for pose in ros_wr.desired_traj_pos])
    velocity_msg = json.dumps([vel.tolist() for vel in ros_wr.desired_traj_vel])
    acceleration_msg = []
    
    message = {"ts": time.time(), "frame_id": "world", "des_position_msg": path_msg, "des_velocity_msg": velocity_msg, "des_acceleration_msg": acceleration_msg}
    web_pub.event = {
            "request": "PUBLISH",
            "type": 'webxr_traj',
            "message": message,
            "channel": web_pub.channel

    }
    print("Trajectory Updated to FRL: n_pos_sample: {}, n_vel_sample: {}".format(len(ros_wr.desired_traj_pos),len(ros_wr.desired_traj_vel) ))
    ros_wr.desired_traj_pos = []
    ros_wr.desired_traj_vel = []
    ros_wr.traj_send = True
    web_pub.traj_updated = True 
    


def main_loop(ros_wr, web_socket_thread, SOCKET):

    web_pub_odom = web_socket_thread[0]
    web_pub_odom2_1 = web_socket_thread[1]
    web_pub_odom2_5 = web_socket_thread[2]
    web_pub_odom2_8 = web_socket_thread[3]
    web_pub_odom2_9 = web_socket_thread[4]
    web_pub_traj = web_socket_thread[5]
    web_pub_static_occupancy = web_socket_thread[6]
    web_pub_final_goal = web_socket_thread[7]
    web_pub_mpl_path = web_socket_thread[8]
    web_sub = web_socket_thread[9]
    webs_task_sub = web_socket_thread[10]
    webs_waypoint_sub = web_socket_thread[11]
    webs_traj_start_sub = web_socket_thread[12]
    webs_take_control_sub = web_socket_thread[13] 

# Initialize ROS

    def spin_ros_node(node):
        rclpy.spin(node)

    # Spin the ROS node in a separate thread
    spin_thread = threading.Thread(target=spin_ros_node, args=(ros_wr,))
    spin_thread.start()

    while rclpy.ok():
        # rclpy.spin_once(ros_wr, timeout_sec=0.1)  # Process ROS callbacks
        if (web_sub.desired_pose_message_received == True):
            #Once the message is received call the publisher 

            print(f"[BACKEND] Received pose message: {web_sub.data_rcv.x}, {web_sub.data_rcv.y}, {web_sub.data_rcv.z}")
            
            ros_wr.publish_des_pose_msg(web_sub)
            web_sub.desired_pose_message_received = False
        
        if (webs_task_sub.desired_task_message_received == True):
            #If the task switching message is received 
            ros_wr.publish_des_task_number(webs_task_sub)
            webs_task_sub.desired_task_message_received = False
       
        if (webs_waypoint_sub.desired_waypoint_list_message_received == True):  
            ros_wr.publish_des_waypoints(webs_waypoint_sub)
            webs_waypoint_sub.desired_waypoint_list_message_received = False

        if (webs_traj_start_sub.start_traj_sig_received == True):
            ros_wr.publish_start_traj_signal(webs_waypoint_sub)
            webs_traj_start_sub.start_traj_sig_received = False
        
        if (webs_take_control_sub.start_take_control_received == True):
            ros_wr.publish_take_control_signal(webs_waypoint_sub)
            webs_take_control_sub.start_take_control_received = False
            
        #Update the web_pub class with the latest odom of the robot 
        update_robot_odom_to_FRL_app(SOCKET, ros_wr, web_pub_odom)
        update_robot_odom_to_FRL_app2_1(SOCKET, ros_wr, web_pub_odom2_1)
        update_robot_odom_to_FRL_app2_5(SOCKET, ros_wr, web_pub_odom2_5)
        update_robot_odom_to_FRL_app2_8(SOCKET, ros_wr, web_pub_odom2_8)
        update_robot_odom_to_FRL_app2_9(SOCKET, ros_wr, web_pub_odom2_9)

        update_static_occupancy_to_FRL_app(SOCKET, ros_wr, web_pub_static_occupancy)
        update_final_goal_to_FRL_app(SOCKET, ros_wr, web_pub_final_goal)
        update_mpl_path_to_FRL_app(SOCKET, ros_wr, web_pub_mpl_path)

        #Update the message related to the desired traj
        update_des_traj_to_FRL_app(ros_wr, web_pub_traj)
        time.sleep(0.1)
        #publisher_to_FRL_app(SOCKET, web_pub, ros_wr)



if __name__ == "__main__":
    rclpy.init(args=None)
    print("ROS2 Initialized")
    ros_wr = ROS_Wrapper()


    host_ = host #128.238.47.55
    port = 8080
    SOCKET = 'ws://'+host_+':8080/'
    print("SOCKET: ", SOCKET)
    
    #Instantiate websocket for subscriber 
    channel_ = "user_position"
    type_ = "SUBSCRIBE"
    webs_sub = webc(channel_, type_)

    #Instantiate websocket for task change subscriber 
    channel_ = "current_task"
    type_ = "SUBSCRIBE"
    webs_task_sub = webc(channel_, type_)
    
    #Instantiate websocket for task change subscriber 
    channel_ = "take_control"
    type_ = "SUBSCRIBE"
    webs_take_control_sub = webc(channel_, type_)

    #Instantiate websocket for waypoints callback subscriber 
    channel_ = "desired_waypoints"
    type_ = "SUBSCRIBE"
    webs_waypoint_sub =  webc(channel_, type_)
    
    #Instantiate websocket for start traje subscriber
    channel_ = "start_traj"
    type_ = "SUBSCRIBE"
    webs_start_traj_sub =  webc(channel_, type_)

    #Instantiate websocket for publisher 
    channel_ = 'robot_odom'
    type_ = "PUBLISH"
    webs_pub =  webc(channel_, type_) #Publishing quadrotor pose for visualization in V

    channel_ = 'robot_odom2_1'
    type_ = "PUBLISH"
    webs_pub2 =  webc(channel_, type_) #Publishing voxl2_1 pose for visualization in V

    channel_ = 'robot_odom2_5'
    type_ = "PUBLISH"
    webs_pub3 =  webc(channel_, type_) #Publishing voxl2_5 pose for visualization in V

    channel_ = 'robot_odom2_8'
    type_ = "PUBLISH"
    webs_pub4 =  webc(channel_, type_) #Publishing voxl2_8 pose for visualization in V

    channel_ = 'robot_odom2_9'
    type_ = "PUBLISH"
    webs_pub5 =  webc(channel_, type_) #Publishing voxl2_9 pose for visualization in V

    channel_ = 'desired_traj'
    type_ = "PUBLISH"
    webs_pub6 =  webc(channel_, type_) #Publishing robot desired trajectory for visualization in V

    channel_ = 'static_occupancy'
    type_ = "PUBLISH"
    webs_pub7 = webc(channel_, type_) #Publishing static occupancy for visualization in V

    channel_ = 'final_goal'
    type_ = "PUBLISH"
    webs_pub8 = webc(channel_, type_) #Publishing final goal for visualization in V

    channel_ = 'mpl_path'
    type_ = "PUBLISH"
    webs_pub9 = webc(channel_, type_) #Publishing final goal for visualization in V

    #Subscriber threads
    wsthread = threading.Thread(target=  subscriber_threading, args = (SOCKET, webs_sub, ros_wr)) #ws.run_forever(sslopt={"cert_reqs": ssl.CERT_NONE}))
    wsthread.start()
    wsthread_sub_task = threading.Thread(target=  subscriber_task_threading, args = (SOCKET, webs_task_sub, ros_wr)) #ws.run_forever(sslopt={"cert_reqs": ssl.CERT_NONE}))
    wsthread_sub_task.start()
    wsthread_sub_waypoints = threading.Thread(target=  subscriber_waypoints_threading, args = (SOCKET, webs_waypoint_sub, ros_wr)) #ws.run_forever(sslopt={"cert_reqs": ssl.CERT_NONE}))
    wsthread_sub_waypoints.start()
    wsthread_sub_traj_start = threading.Thread(target=  subscriber_start_traj_threading, args = (SOCKET, webs_start_traj_sub, ros_wr)) #ws.run_forever(sslopt={"cert_reqs": ssl.CERT_NONE}))
    wsthread_sub_traj_start.start()
    wsthread_sub_take_control = threading.Thread(target=  subscriber_take_control_threading, args = (SOCKET, webs_take_control_sub, ros_wr)) #ws.run_forever(sslopt={"cert_reqs": ssl.CERT_NONE}))
    wsthread_sub_take_control.start()

    

    wsthread2 = threading.Thread(target=  publisher_to_FRL_app_threading, args = (SOCKET, webs_pub, ros_wr)) 
    wsthread2.start()

    wsthread3 = threading.Thread(target=  publisher_to_FRL_app_threading2_1, args = (SOCKET, webs_pub2, ros_wr)) 
    wsthread3.start()

    wsthread4 = threading.Thread(target=  publisher_to_FRL_app_threading2_5, args = (SOCKET, webs_pub3, ros_wr)) 
    wsthread4.start()

    wsthread5 = threading.Thread(target=  publisher_to_FRL_app_threading2_8, args = (SOCKET, webs_pub4, ros_wr)) 
    wsthread5.start()

    wsthread6 = threading.Thread(target=  publisher_to_FRL_app_threading2_9, args = (SOCKET, webs_pub5, ros_wr)) 
    wsthread6.start()

    wsthread7 = threading.Thread(target=  publisher_to_FRL_app_robot_path, args = (SOCKET, webs_pub6, ros_wr)) 
    wsthread7.start()

    wsthread8 = threading.Thread(target=  publisher_to_FRL_app_static_occupancy, args = (SOCKET, webs_pub7, ros_wr))
    wsthread8.start()

    wsthread9 = threading.Thread(target=  publisher_to_FRL_app_final_goal, args = (SOCKET, webs_pub8, ros_wr))
    wsthread9.start()

    wsthread10 = threading.Thread(target=  publisher_to_FRL_app_mpl_path, args = (SOCKET, webs_pub9, ros_wr))
    wsthread10.start()


    #Instantiate a main thread that has the ability to access the data of the various classes 
    web_socket_thread = [webs_pub,webs_pub2, webs_pub3, webs_pub4, webs_pub5, webs_pub6, webs_pub7, webs_pub8, webs_pub9, webs_sub, webs_task_sub, webs_waypoint_sub, webs_start_traj_sub, webs_take_control_sub]
    wsthread_main = threading.Thread(target=  main_loop, args = (ros_wr,web_socket_thread, SOCKET)) 
    wsthread_main.start()


