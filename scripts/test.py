#!/usr/bin/env python
from __future__ import print_function, division
import rospy
import tf2_ros
import geometry_msgs.msg
from geometry_msgs.msg import Twist
import math
from robomaster_robot import robomaster_robot

class RobotTester(object): 
    def __init__(self):
        rospy.init_node('robot_tester', anonymous=True)
        robot_number = 0
        init_x = 0
        init_y = 0
        self.robot0 = robomaster_robot(robot_number, init_x, init_y)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.movement_distance = rospy.get_param('~movement_distance', 1)

        self.directions = [
            (1, 0),    # Right
            (-1, 0),   # Left
            (0, 1),    # Forward
            (0, -1),   # Backward
            (1/math.sqrt(2), 1/math.sqrt(2)),    # Forward-Right
            (1/math.sqrt(2), -1/math.sqrt(2)),   # Backward-Right
            (-1/math.sqrt(2), 1/math.sqrt(2)),   # Forward-Left
            (-1/math.sqrt(2), -1/math.sqrt(2)),  # Backward-Left
        ]

        self.Kp = rospy.get_param('~Kp', 0.5)
        self.max_speed = rospy.get_param('~max_speed', 0.5)
        self.rate = rospy.Rate(10)

    def send_velocities(self, vx, vy):
        self.robot0.send_velocities(vx, vy)

    def get_robot_position(self):
        try:
            trans = self.tf_buffer.lookup_transform('world', 'robot0_odom_combined', rospy.Time())
            return trans.transform.translation.x, trans.transform.translation.y
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("Failed to get robot position, using (0, 0)")
            return 0, 0

    def move_robot(self, target_x, target_y):
        start_x, start_y = self.get_robot_position()
        target_x += start_x
        target_y += start_y

        while not rospy.is_shutdown():
            current_x, current_y = self.get_robot_position()

            error_x = target_x - current_x
            error_y = target_y - current_y

            vx = self.Kp * error_x
            vy = self.Kp * error_y

            # Limiting speed
            speed = math.sqrt(vx**2 + vy**2)
            if speed > self.max_speed:
                vx = vx / speed * self.max_speed
                vy = vy / speed * self.max_speed

            self.send_velocities(vx, vy)

            if abs(error_x) < 0.01 and abs(error_y) < 0.01:
                break

            self.rate.sleep()

        self.stop_robot()

    def stop_robot(self):
        self.robot0.send_velocities(0, 0)

    def run_test(self):
        for i, (x, y) in enumerate(self.directions):
            print("Testing direction {}: ({}, {})".format(i+1, x, y))
            for j in range(10):
                print("Movement {} forward:".format(j+1))
                self.move_robot(x * self.movement_distance, y * self.movement_distance)
                raw_input("Press Enter after measuring to start backward movement")
                self.move_robot(-x * self.movement_distance, -y * self.movement_distance)
                raw_input("Press Enter after measuring to continue")

            print("Finished testing this direction.")
            raw_input("Press Enter to move to the next direction")

    def shutdown(self):
        self.stop_robot()

if __name__ == '__main__':
    tester = None
    try:
        tester = RobotTester()
        tester.run_test()
    except rospy.ROSInterruptException:
        pass
    finally:
        if tester is not None:
            tester.shutdown()



