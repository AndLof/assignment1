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


def quaternion_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class Rotate360(Node):

    def __init__(self):
        super().__init__("rotate_360_node")

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.image_pub = self.create_publisher(Image, "/camera/circle_marker", 10)

        self.bridge = CvBridge()

        self.current_yaw = None
        self.initial_yaw = None

        self.current_image = None
        self.current_markers = []

        self.rotation_strarting_flag = 0

        self.detected_ids = []
        self.detected_yaws = []

        self.create_subscription(Odometry, "/odom", self.odom_callback, 50)
        self.create_subscription(ArucoDetection, "/aruco_detections", self.aruco_callback, 10)
        self.create_subscription(Image, "/camera/image", self.image_callback, 10)

        # CAMERA PARAMS (costanti)
        self.fx = 381.36
        self.fy = 381.36
        self.marker_size = 0.20  # lato marker [m]

    # ---------------- CALLBACK ---------------- #

    def odom_callback(self, msg):
        yaw = quaternion_to_yaw(msg.pose.pose.orientation)

        if self.initial_yaw is None:
            self.initial_yaw = yaw

        self.current_yaw = yaw

        if abs(self.initial_yaw - self.current_yaw) > 0.20:
            self.rotation_strarting_flag = 1

    def aruco_callback(self, msg):
        self.current_markers = msg.markers

        for m in msg.markers:
            if m.marker_id not in self.detected_ids:
                self.detected_ids.append(m.marker_id)
                self.detected_yaws.append(self.current_yaw)

    def image_callback(self, msg):
        self.current_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    # ---------------- ROTATE ---------------- #

    def rotate(self, angular_speed):
        #flag_for_starting = 1
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.z = angular_speed
        print(angular_speed)
        rclpy.spin_once(self, timeout_sec = 2)
        if ((self.current_yaw== None) or (self.initial_yaw == None)):
            print("Problem")
            rclpy.spin_once(self, timeout_sec = 5)


        # pubblica per tutta la durata
        while rclpy.ok() and ((abs(self.current_yaw - self.initial_yaw) > 0.17) or (self.rotation_strarting_flag == 0)):
            #if abs(self.current_yaw - self.initial_yaw) > 0.3:
            #    flag_for_starting = 0
            self.cmd_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec = 0.05)

        # stop
        msg.angular.z = 0.0
        self.cmd_pub.publish(msg)
        self.get_logger().info(f"[DETECT] Marker {self.detected_ids}")

    # ---------------- SPOTTING ---------------- #

    def spotting(self):

        self.detected_ids.sort()

        for marker_id in self.detected_ids:

            self.get_logger().info(f"Spotting marker {marker_id}")

            marker = None

            # --- CERCA MARKER RUOTANDO ---
            while rclpy.ok() and marker is None:
                rclpy.spin_once(self, timeout_sec=0.05)

                for m in self.current_markers:
                    if m.marker_id == marker_id:
                        marker = m
                        break

                if marker is None:
                    t = Twist()
                    t.angular.z = 0.3
                    self.cmd_pub.publish(t)

            self.cmd_pub.publish(Twist())
            time.sleep(0.3)

            # --- CENTRA ORIZZONTALMENTE ---
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.05)

                error_x = marker.pose.position.x
                if abs(error_x) < 0.01:
                    break

                t = Twist()
                t.angular.z = -1.5 * error_x
                self.cmd_pub.publish(t)

                # aggiorna marker
                for m in self.current_markers:
                    if m.marker_id == marker_id:
                        marker = m
                        break

            self.cmd_pub.publish(Twist())

            # --- DISEGNA CERCHIO ---
            img = self.current_image.copy()
            h, w, _ = img.shape
            cx, cy = w // 2, h // 2

            X = marker.pose.position.x
            Y = marker.pose.position.y
            Z = marker.pose.position.z

            u = int(self.fx * X / Z + cx)
            v = int(self.fy * Y / Z + cy)

            R = int(self.fx * (self.marker_size * math.sqrt(2) / 2) / Z)

            cv2.circle(img, (u, v), R, (0, 0, 255), 2)

            self.image_pub.publish(
                self.bridge.cv2_to_imgmsg(img, "bgr8")
            )

            cv2.imshow("Circle Marker", img)
            cv2.waitKey(1)

            time.sleep(1.0)



def main():
    rclpy.init()
    node = Rotate360()
    node.rotate(0.5)
    node.spotting()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
