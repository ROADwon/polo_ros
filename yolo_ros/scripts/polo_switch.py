#! /usr/bin/env/ python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from yolo_msgs.msg import Inference

import threading
import time

class TFSwitch(Node):
    def __init__(self):
        super().__init__('tf_switch')
        self.get_logger().info("TF Switch Node is UP!")
        self.subscription = self.create_subscription(
            Inference,
            'yolo/inference',
            self.inference_callback,
            10
        )
        self.get_logger().info("Subscribe to yolo/inference topic")

        self.twist_pub = self.create_publisher(Twist, 'cmd_vel_invade', 10)

        self.invasion_detected = False
        self.lock = threading.Lock()
        self.pub_thread = threading.Thread(target=self.publish_twist)
        self.pub_thread.daemon = True
        self.pub_thread.start()

    def inference_callback(self, msg):
        with self.lock:
            self.invasion_detected = any(result.class_name == "INVADE" for result in msg.yolo_inference)

    def publish_twist(self):
        rate = self.create_rate(15) # 15Hz
        while rclpy.ok():
            with self.lock:
                if self.invasion_detected:
                    twist = Twist()
                    twist.linear.x = 0.0
                    twist.angular.z = 0.5
                    self.twist_pub.publish(twist)
                    self.get_logger().info("INVADE detected, Publishing Twist msg")
                else:
                    self.get_logger().info("No INVADE detected")
            rate.sleep()

def main(args=None):
    rclpy.init(args=args)
    tf_switch = TFSwitch()
    rclpy.spin(tf_switch)
    tf_switch.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
