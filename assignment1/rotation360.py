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
        self.image_pub = self.create_publisher(Image, 'camera/circle_marker', 10)
        self.initial_yaw = None
        self.current_yaw = None

        self.bridge = CvBridge()
        self.current_image = None #?
        self.current_markers = [] #?

        self.odom_sub = self.create_subscription(Odometry, "/odom", self.odom_callback, 100)
        self.aruco_sub = self.create_subscription(ArucoDetection, "/aruco_detections", self.aruco_callback, 10)
        self.image_sub = self.create_subscription(Image, "/camera/image", self.image_callback, 10)
        
        #self.get_logger().info("Avvio rotazione di 360°...")
        #self.markers_ID[5] = [0, 0, 0, 0, 0]
        self.rotation_strarting_flag = 0

        self.detected_ids = []
        self.detected_yaws = []


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

    def image_callback(self, msg):
        self.current_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
                    

    def spotting(self):

        #RIORDINO ID E YAW
        if len(self.detected_ids) == 0:
            self.get_logger().info("No markers detected")
            return

        sorted_pairs = sorted(zip(self.detected_ids, self.detected_yaws))
        self.detected_ids, self.detected_yaws = map(list, zip(*sorted_pairs))

        #CICLO SUI MARKER (DAL PIÙ PICCOLO)
        for marker_id, target_yaw in zip(self.detected_ids, self.detected_yaws):

            self.get_logger().info(f"Spotting marker {marker_id}")

            #ROTAZIONE FINO ALLO YAW DEL MARKER
            while rclpy.ok():
                error = target_yaw - self.current_yaw

                while error > math.pi:
                    error -= 2.0 * math.pi
                while error < -math.pi:
                    error += 2.0 * math.pi

                if abs(error) < 0.03:
                    break

                msg = Twist()
                msg.angular.z = 1.0 * error
                self.publisher_.publish(msg)
                rclpy.spin_once(self, timeout_sec=0.05)

            self.publisher_.publish(Twist())
            time.sleep(0.5)

            #CENTRO ORIZZONTALMENTE IL MARKER (POSE 3D)
            while rclpy.ok():
                marker = None
                for m in self.current_markers:
                    if m.marker_id == marker_id:
                        marker = m
                        break

                if marker is None:
                    break

                error_x = marker.pose.position.x

                if abs(error_x) < 0.01:
                    break

                msg = Twist()
                msg.angular.z = -1.5 * error_x
                self.publisher_.publish(msg)
                rclpy.spin_once(self, timeout_sec=0.05)

            self.publisher_.publish(Twist())
            time.sleep(0.5)

            #DISEGNO CERCHIO ROSSO
            if self.current_image is None or marker is None:
                continue

            # PARAMETRI CAMERA / MARKER
            fx = 381.36          # focale: preso da primo elemento della matrice k in camera_info
            marker_size = 0.20  # lato marker in metri?

            R_px = int((fx * marker_size) / (2.0 * marker.pose.position.z))

            h, w, _ = self.current_image.shape
            center = (w // 2, h // 2)

            img_out = self.current_image.copy()
            cv2.circle(img_out, center, R_px, (0, 0, 255), 2)

            #PUBBLICAZIONE IMMAGINE
            img_msg = self.bridge.cv2_to_imgmsg(img_out, encoding="bgr8")
            self.image_pub.publish(img_msg)

            #POPUP IMMAGINE
            cv2.imshow("Circle Marker", img_out)
            cv2.waitKey(1)

            time.sleep(1.0)




def main(args=None):
    rclpy.init(args=args)
    node = Rotate360()
    #rclpy.spin_once(node, timeout_sec = 1)
    node.rotate(0.5)
    node.spotting()
    #rclpy.shutdown()
    #node.destroy_node()
    


if __name__ == "__main__":
    main()
