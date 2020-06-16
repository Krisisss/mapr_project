#!/usr/bin/env python
import rospy as rp
from visualization_msgs.msg import Marker
from tf.transformations import *



class Car:
    def __init__(self, x, y, theta, name, color):
        self.pub = rp.Publisher('point_' + name, Marker, queue_size=10)
        self.marker = Marker()

        self.marker.header.frame_id = "map"
        self.marker.header.stamp = rp.Time.now()
        self.marker.ns = name
        self.marker.id = 0
        self.marker.type = Marker.ARROW
        self.marker.pose.position.x = x
        self.marker.pose.position.y = y
        self.marker.pose.position.z = 0.005


        q_rot = quaternion_from_euler(0, 0, theta)

        self.marker.pose.orientation.x = q_rot[0]
        self.marker.pose.orientation.y = q_rot[1]
        self.marker.pose.orientation.z = q_rot[2]
        self.marker.pose.orientation.w = q_rot[3]
        self.marker.scale.x = 0.025
        self.marker.scale.y = 0.01
        self.marker.scale.z = 0.005

        self.marker.color.r = color[0]
        self.marker.color.g = color[1]
        self.marker.color.b = color[2]
        self.marker.color.a = 0.5

    def publish(self):
        self.pub.publish(self.marker)

if __name__ == '__main__':
    rp.init_node('points', log_level=rp.DEBUG)
    while not rp.is_shutdown():
        rp.sleep(0.5)
