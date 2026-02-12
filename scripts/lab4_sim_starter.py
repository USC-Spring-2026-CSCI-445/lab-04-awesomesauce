#!/usr/bin/env python3
from math import inf
import queue
from time import sleep, time

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

MAX_ROT_VEL = 2.84 # rad/s
MAX_LIN_VEL = 0.22 # m/s

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
        self.t_prev = None

    def clamp(self, output):
        if output < self.u_min:
            return self.u_min
        elif output > self.u_max:
            return self.u_max
        else:
            return output

    def control(self, err, t):
        if (self.t_prev is None):
            self.t_prev = t
            return 0

        dt = t - self.t_prev
        self.t_prev = t

        if dt <= rospy.Duration.from_sec(1e-10):
            return 0

        # Compute control action
        output = self.kP * err
        return self.clamp(output)


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
        self.t_prev =  None
        self.e_prev = 0.0

    def clamp(self, output):
        if output < self.u_min:
            return self.u_min
        elif output > self.u_max:
            return self.u_max
        else:
            return output

    def control(self, err, t):
        if (self.t_prev is None):
            self.t_prev = t
            return 0

        dt = t - self.t_prev
        self.t_prev = t

        if dt <= rospy.Duration.from_sec(1e-10):
            return 0

        # Compute control action
        de = err - self.e_prev
        output = (self.kP * err) + (self.kD * (de/dt.to_sec()))
        return self.clamp(output)


class RobotController:
    def __init__(self, desired_distance: float):
        print("\nMake the robot follow the wall on its left by maintaining the distance from it using LIDAR.\n")

        # ROS1 infrastructure
        rospy.init_node("robot_controller", anonymous=True)
        self.laserscan_sub = rospy.Subscriber("/scan", LaserScan, self.robot_laserscan_callback)
        self.robot_ctrl_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        # Define PD controller for wall-following here
        self.rot_controller = PDController(0.3, 0.0, -1 * MAX_ROT_VEL, MAX_ROT_VEL)
        self.lin_controller = PDController(1, 0.01, 0.00, MAX_LIN_VEL)

        self.desired_distance = desired_distance  # Desired distance from the wall
        self.ir_distance = None
        

    def robot_laserscan_callback(self, lscan: LaserScan):
        left = lscan.ranges[80:100]
        left = [x for x in left if x != inf]
        if len(left) > 0:
            self.ir_distance = sum(left) / len(left)

    def control_loop(self):

        rate = rospy.Rate(20)

        while not rospy.is_shutdown():

            if self.ir_distance is None:
                print("Waiting for IR sensor readings")
                sleep(0.1)
                continue

            ctrl_msg = Twist()

            err = desired_distance - self.ir_distance

            # using PD controller, compute and send motor commands
            u = self.rot_controller.control(err, rospy.get_rostime())
            ctrl_msg.angular.z = -1 * u
            v = self.lin_controller.control(err, rospy.get_rostime())
            ctrl_msg.linear.x = MAX_LIN_VEL - v

            self.robot_ctrl_pub.publish(ctrl_msg)
            print(f"dist: {round(self.ir_distance, 4)}\ttgt: {round(self.desired_distance, 4)}\tu: {round(u, 4)}\tv: {round(v, 4)}")
            rate.sleep()


if __name__ == "__main__":
    desired_distance = 0.7
    controller = RobotController(desired_distance)
    try:
        controller.control_loop()
    except rospy.ROSInterruptException:
        pass
