#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import math
import time

class RobotTester:
    def __init__(self):
        rospy.init_node('robot_tester', anonymous=True)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(10)

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

        self.Kp = 0.5
        self.max_speed = 0.5

    def send_velocities(self, vx, vy):
        twist = Twist()
        twist.linear.x = vx
        twist.linear.y = vy
        self.pub.publish(twist)

    def move_robot(self, target_x, target_y):
        start_time = rospy.Time.now()
        current_x, current_y = 0, 0

        while not rospy.is_shutdown():
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

            dt = (rospy.Time.now() - start_time).to_sec()
            current_x += vx * dt
            current_y += vy * dt
            start_time = rospy.Time.now()

            if abs(error_x) < 0.01 and abs(error_y) < 0.01:
                break

            self.rate.sleep()

        self.stop_robot()

    def stop_robot(self):
        self.send_velocities(0, 0)

    def run_test(self):
        for i, (x, y) in enumerate(self.directions):
            print(f"Testing direction {i+1}: ({x}, {y})")
            for j in range(10):
                print(f"Movement {j+1} forward:")
                input("Press Enter to start forward movement")
                self.move_robot(x, y)
                input("Press Enter after measuring to start backward movement")
                self.move_robot(-x, -y)
                input("Press Enter after measuring to continue")

            print("Finished testing this direction.")
            input("Press Enter to move to the next direction")

    def shutdown(self):
        self.stop_robot()

if __name__ == '__main__':
    try:
        tester = RobotTester()
        tester.run_test()
    except rospy.ROSInterruptException:
        pass
    finally:
        tester.shutdown()