#! /usr/bin/env python3

import cv2 as cv
import numpy as np
import pandas as pd
import torch

import os
import timeit
import sklearn
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge

from ultralytics import YOLO
from collections import Counter, Deque
from shapely.geometry import Polygon, Point
from yolo_msgs.msg import Inference
from yolo_msgs.msg import InferenceResult

bridge = CvBridge()

if torch.cuda.is_available():
    device = torch.device('cuda')
else :
    device = torch.device('cpu')

class CameraSubscriber(Node):

    def __init__(self):
        super().__init__('camera_subscriber')
        self.get_logger().info('Camera Subscriber Node has been started')
        self.model = YOLO('yolov8n.pt')
        self.model.to(device)
        self.yolo_inference = Inference()

        self.subscription = self.create_subscription(
            Image,
            'camera/color/image_raw',
            self.listener_callback,
            10)
        self.subscription

        self.get_logger().info('Subscribed to camera topic')

        #published message
        self.yolo_pub = self.create_publisher(Inference, 'yolo/inference', 10)
        self.img_pub = self.create_publisher(Image, 'yolo/image', 10)
        self.twist_pub = self.create_publisher(Twist, 'cmd_vel_invade', 10)

        self.points = [(270,270), (435, 270), (435, 480), (270, 480)]
        self.points_arr = np.array(self.points)

        self.polygon = Polygon(self.points)

    def points_in_polygon(self, points):
        point = Point(points)
        return self.polygon.contains(point)

    def camera_callback(self, data):
        self.yolo_inference.header.frame_id = 'inference'
        self.yolo_inference.header.stamp = self.get_clock().now().to_msg()

        img = bridge.imgmsg_to_cv2(data, 'bgr8')

        results = self.model(img)

        for result in results:
            boxes = result.boxes
            for box in boxes:
                self.inference_result = InferenceResult()

                r = box.xyxy[0].cpu().numpy().astype(int)
                ct = box.xywh[0].cpu().numpy().astype(int)

                invade_check_point = ((r[0] + r[2])/2, r[3])
                inside_polygon = self.points_in_polygon(invade_check_point)
                if inside_polygon :
                    class_name = "invade"
                    cv.rectangle(img, (r[0], r[1]), (r[2], r[3]), (0, 255, 0), 2)
                    confi = np.round(box.conf[0].cpu().numpy(), 4)
                    cv.putText(img, f'{class_name} {confi}', (r[0], r[1]), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    cv.circle(img, (int(invade_check_point[0]), int(invade_check_point[1])), 5, (0, 255, 0), -1)

                    self.inference_result.class_name = class_name
                    self.inference_result.top = int(r[0])
                    self.inference_result.left = int(r[1])
                    self.inference_result.bottom = int(r[2])
                    self.inference_result.right = int(r[3])

                    self.get_logger().info(f'Invasion Detected at {invade_check_point}')
                    self.yolo_inference.results.append(self.inference_result)

                    twist = Twist()
                    twist.linear.x = 0.0
                    twist.linear.y = 0.0
                    twist.linear.z = 0.0
                    twist.angular.x = 0.0
                    twist.angular.y = 0.0
                    twist.angular.z = 0.0
                    self.twist_pub.publish(twist)
                    self.get_logger().info("Emergency Stop")

        cv.polylines(img, [self.points_arr.astype(np.int32)], True, (0, 255, 0), 2)
        img_msg = bridge.cv2_to_imgmsg(img, 'bgr8')

        self.img_pub.publish(img_msg)
        self.yolo_pub.publish(self.yolo_inference)
        self.yolo_inference.results.clear()


if __name__ == '__main__':
    rclpy.init()
    camera_subscriber = CameraSubscriber()
    rclpy.spin(camera_subscriber)
    camera_subscriber.destroy_node()
    rclpy.shutdown()











