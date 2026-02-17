#!/usr/bin/env python3
from math import inf
import queue
from time import sleep, time

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


# P controller class
class PController:
    """
    Generates control action taking into account instantaneous error (proportional action).
    """

    def __init__(self, kP, u_min, u_max):
        assert u_min < u_max, "u_min should be less than u_max"
        # Initialize variables here
        ######### Your code starts here #########
        self.kP = kP
        self.u_min = u_min
        self.u_max = u_max 
        self.t_prev = time
        ######### Your code ends here #########

    def control(self, err, t):
        dt = t - self.t_prev
        if dt <= 1e-6:
            return 0

        # Compute control action here
        ######### Your code starts here #########
        u = self.kP * err #multiplying by dt would make it act like an integrator
        u = max(u, self.u_min)
        u = min(u, self.u_max)
        return u
        ######### Your code ends here #########


# PD controller class
class PDController:
    """
    Generates control action taking into account instantaneous error (proportional action)
    and rate of change of error (derivative action).
    """

    def __init__(self, kP, kD, u_min, u_max):
        assert u_min < u_max, "u_min should be less than u_max"
        # Initialize variables here
        ######### Your code starts here #########
        self.kP = kP
        self.kD = kD
        self.u_min = u_min
        self.u_max = u_max
        self.t_prev = 0
        self.err_prev = 0
        ######### Your code ends here #########

    def control(self, err, t):
        dt = t - self.t_prev
        if dt <= 1e-6:
            return 0

        # Compute control action here
        ######### Your code starts here #########
        u = self.kP * err + self.kD * (err - self.err_prev) / dt
        u = max(u, self.u_min)
        u = min(u, self.u_max)
        self.t_prev = t
        self.err_prev = err
        return u
        ######### Your code ends here #########


class RobotController:
    def __init__(self, desired_distance: float):
        print("\nMake the robot follow the wall on its left by maintaining the distance from it using LIDAR.\n")

        # ROS1 infrastructure
        rospy.init_node("robot_controller", anonymous=True)
        self.laserscan_sub = rospy.Subscriber("/scan", LaserScan, self.robot_laserscan_callback)
        self.robot_ctrl_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        # Define _PD_ controller for wall-following here
        ######### Your code starts here #########
        self.angular_controller = PDController(2.0, 0.1, -2.84, 2.84)
        self.base_velocity = 0.08
        ######### Your code ends here #########

        self.desired_distance = desired_distance  # Desired distance from the wall
        self.ir_distance = None #distance = 1597 * pow(raw,-1.522)

    def robot_laserscan_callback(self, lscan: LaserScan):
        # Left side (wall following)
        left = lscan.ranges[80:100]
        left = [x for x in left if x != inf]
        if len(left) > 0:
            self.ir_distance = sum(left) / len(left)
        
        # Front (obstacle avoidance)
        front = lscan.ranges[0:15] + lscan.ranges[345:360]
        front = [x for x in front if x != inf]
        self.front_distance = min(front) if len(front) > 0 else inf

    def control_loop(self):

        rate = rospy.Rate(20)

        while not rospy.is_shutdown():

            if self.ir_distance is None:
                print("Waiting for IR sensor readings")
                sleep(0.1)
                continue

            ctrl_msg = Twist()

            # using PD controller, compute and send motor commands
            ######### Your code starts here #########
            t = time()
            err = self.desired_distance - self.ir_distance
            u = self.angular_controller.control(err, t)

            # Corner handling: if wall ahead, turn right harder and slow down
            if self.front_distance < 0.5:
                ctrl_msg.linear.x = 0.05
                ctrl_msg.angular.z = -1.5  # turn right away from front wall
            else:
                ctrl_msg.linear.x = self.base_velocity
                ctrl_msg.angular.z = -u
            ######### Your code ends here #########

            self.robot_ctrl_pub.publish(ctrl_msg)
            print(f"dist: {round(self.ir_distance, 4)}\ttgt: {round(self.desired_distance, 4)}\tu: {round(u, 4)}")
            rate.sleep()


if __name__ == "__main__":
    desired_distance = 0.5
    controller = RobotController(desired_distance)
    try:
        controller.control_loop()
    except rospy.ROSInterruptException:
        pass
