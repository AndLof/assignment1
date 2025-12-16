#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from aruco_opencv_msgs.msg import ArucoDetection
import math
import time

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

#from tf_transformations import euler_from_quaternion
def quaternion_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class Rotate360(Node):
    def __init__(self):
        super().__init__("rotate_360_node")
        self.publisher_ = self.create_publisher(Twist, "cmd_vel", 10)
        #self.image_pub = self.create_publisher(Image, 'camera/circle_marker', 10)
        self.initial_yaw = None
        self.current_yaw = None
        self.odom_sub = self.create_subscription(Odometry, "/odom", self.odom_callback, 10)
        self.aruco_sub = self.create_subscription(ArucoDetection, "/aruco_detections", self.aruco_callback, 10)
        #self.image_sub = self.create_subscription(Image, "/camera/image_raw", self.image_callback, 10)
        
        #self.get_logger().info("Avvio rotazione di 360°...")
        #self.markers_ID[5] = [0, 0, 0, 0, 0]
        self.rotation_strarting_flag = 0

        self.detected_ids = []
        self.detected_yaws = []

        #self.bridge = CvBridge()
        #self.current_image = None #?
        #self.current_markers = [] #?

    def rotate(self, angular_speed):
        #flag_for_starting = 1
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.z = angular_speed
        print(angular_speed)
        rclpy.spin_once(self, timeout_sec = 2)


        # pubblica per tutta la durata
        while rclpy.ok() and ((abs(self.current_yaw - self.initial_yaw) > 0.17) or (self.rotation_strarting_flag == 0)):
            #if abs(self.current_yaw - self.initial_yaw) > 0.3:
            #    flag_for_starting = 0
            self.publisher_.publish(msg)
            rclpy.spin_once(self, timeout_sec = 0.05)

        # stop
        msg.angular.z = 0.0
        self.publisher_.publish(msg)
        self.get_logger().info(f"[DETECT] Marker {self.detected_ids}")


    def odom_callback(self, msg):
        # Estrai yaw dall'orientamento
        orientation_q = msg.pose.pose.orientation
        """
        quaternion = (
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w,
        )
        _, _, yaw = euler_from_quaternion(quaternion)
        """
        yaw = quaternion_to_yaw(orientation_q)


        if self.initial_yaw == None:
            self.initial_yaw = yaw
        self.current_yaw = yaw
        if abs(self.initial_yaw - self.current_yaw) > 0.20:
            self.rotation_strarting_flag = 1

    def aruco_callback(self, msg: ArucoDetection):

        for marker in msg.markers:
            marker_id = marker.marker_id

            if marker_id not in self.detected_ids:
                self.detected_ids.append(marker_id)
                self.detected_yaws.append(self.current_yaw)
                #self.get_logger().info(f"[DETECT] Marker {marker_id}")

    #def image_callback(self, msg):
    #    self.current_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
                    




def main(args=None):
    rclpy.init(args=args)
    node = Rotate360()
    #rclpy.spin_once(node, timeout_sec = 1)
    node.rotate(0.5)
    #rclpy.shutdown()
    #node.destroy_node()
    


if __name__ == "__main__":
    main()
