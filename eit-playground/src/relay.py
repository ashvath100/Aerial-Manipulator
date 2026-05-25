#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped

from mavros.base import SENSOR_QOS


class RelayNode(Node):

    def __init__(self):
        super().__init__('relay')

        self.subscription = self.create_subscription(PoseStamped,'/vrpn_mocap/Drone/pose',self.listener_callback,SENSOR_QOS)
        self.subscription  # prevent unused variable warning

        # self.publisher = self.create_publisher(PoseStamped, '/mavros/mocap/pose', SENSOR_QOS)
        self.publisher = self.create_publisher(PoseStamped, '/mavros/vision_pose/pose', SENSOR_QOS)
        #timer_period = 0.5  # seconds
        #self.timer = self.create_timer(timer_period, self.timer_callback)
        #self.i = 0

    def listener_callback(self, msg):
        #self.get_logger().info('I heard: "%s"' % msg)
        msg.header.frame_id = "map"
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    relay_node = RelayNode()

    rclpy.spin(relay_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    relay_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
