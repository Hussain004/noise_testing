#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist, TransformStamped
import tf2_ros

class robomaster_robot:
    def __init__(self, no, init_x = 0, init_y = 0):
        self.name = "robot{}".format(no)
        # rospy.init_node("{}_controller".format(self.name))
        rospy.Subscriber("{}/robot_pose_ekf/odom_combined".format(self.name), PoseWithCovarianceStamped, self.update_pose)
        self.pub = rospy.Publisher("{}/cmd_vel".format(self.name), Twist, queue_size=10)
        self.init_x = init_x
        self.init_y = init_y

    def update_pose(self, data):
        broadcaster = tf2_ros.TransformBroadcaster()
        t = TransformStamped()

        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "world"
        t.child_frame_id = "{}_odom_combined".format(self.name)
        t.transform.translation.x = data.pose.pose.position.x + self.init_x
        t.transform.translation.y = data.pose.pose.position.y + self.init_y
        t.transform.translation.z = 0.0
        t.transform.rotation = data.pose.pose.orientation

        broadcaster.sendTransform(t)
        # self.yaw, _, __ =  euler_from_quaternion(data.pose.pose.orientation)

    def send_velocities(self, vx, vy, omega=0):
        sending = Twist()
        sending.linear.x = vx
        sending.linear.y = vy
        sending.angular.z = omega
        self.pub.publish(sending)
