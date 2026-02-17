#!/usr/bin/env python3
import queue
from time import sleep, time

import rospy
from geometry_msgs.msg import Twist
from turtlebot3_msgs.msg import SensorState


# P controller class
class PController:
    """
    Generates control action taking into account instantaneous error (proportional action).
    """

    def __init__(self, kP, u_min, u_max):
        assert u_min < u_max, "u_min should be less than u_max"
        self.kP = kP
        self.u_min = u_min
        self.u_max = u_max
        self.t_prev = time()

    def control(self, err, t):
        dt = t - self.t_prev
        if dt <= 1e-6:
            return 0

        u = self.kP * err
        u = max(u, self.u_min)
        u = min(u, self.u_max)
        self.t_prev = t
        return u


# PD controller class
class PDController:
    """
    Generates control action taking into account instantaneous error (proportional action)
    and rate of change of error (derivative action).
    """

    def __init__(self, kP, kD, u_min, u_max):
        assert u_min < u_max, "u_min should be less than u_max"
        self.kP = kP
        self.kD = kD
        self.u_min = u_min
        self.u_max = u_max
        self.t_prev = time()
        self.err_prev = 0

    def control(self, err, t):
        dt = t - self.t_prev
        if dt <= 1e-6:
            return 0

        u = self.kP * err + self.kD * (err - self.err_prev) / dt
        u = max(u, self.u_min)
        u = min(u, self.u_max)
        self.t_prev = t
        self.err_prev = err
        return u


class RobotController:
    def __init__(self, desired_distance: float):
        print("\nMake the robot follow the wall on its right by maintaining the distance from it using IR sensor.\n")

        # ROS1 infrastructure
        rospy.init_node("robot_controller", anonymous=True)
        self.cliff_sub = rospy.Subscriber("/sensor_state", SensorState, self.sensor_state_callback, queue_size=1)
        self.robot_ctrl_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        # Same gains as sim — works because distance equation is scaled to same range
        ######### Your code starts here #########
        self.angular_controller = PController(2.0, 0.5, -2.84, 2.84)
        self.base_velocity = 0.12
        ######### Your code ends here #########

        self.desired_distance = desired_distance
        self.ir_distance = None

    def sensor_state_callback(self, state: SensorState):
        raw = state.cliff
        ######### Your code starts here #########
        # Scale raw to match sim distance range (~0.2 to ~0.8m)
        # Based on observed data:
        #   raw 530 (closest to wall)  -> 0.20m
        #   raw 475 (near wall)        -> 0.37m
        #   raw 440 (medium)           -> 0.48m
        #   raw 400 (far from wall)    -> 0.61m
        #   raw 370 (very far)         -> 0.70m
        # Below raw 350: sensor is out of range (corner), treat as very far
        if raw > 350:
            distance = -0.003125 * raw + 1.856
            distance = max(distance, 0.05)
        else:
            distance = float('inf')
        ######### Your code ends here #########
        self.ir_distance = distance

    def control_loop(self):

        rate = rospy.Rate(20)

        while not rospy.is_shutdown():

            if self.ir_distance is None:
                print("Waiting for IR sensor readings")
                sleep(0.1)
                continue

            ctrl_msg = Twist()

            ######### Your code starts here #########
            t = time()

            # err = measured - desired
            # Too far from right wall (dist big):  err > 0, u > 0, angular.z < 0 -> turn RIGHT toward wall
            # Too close to right wall (dist small): err < 0, u < 0, angular.z > 0 -> turn LEFT away from wall
            # Corner (dist = inf):                  err = inf, u = +2.84, angular.z = -2.84 -> hard RIGHT
            err = self.ir_distance - self.desired_distance
            u = self.angular_controller.control(err, t)

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
