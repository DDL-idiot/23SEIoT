#!/usr/bin/env python3
import rospy
import math
import actionlib
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import PoseStamped

rospy.init_node('goal_pose')

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.integral = 0

    def compute(self, error):
        self.integral += error
        derivative = error - self.prev_error
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return output

def move(init_pose, goal_pose, pid_linear, pid_angular):
    rate = rospy.Rate(10)  
    while not check_goal_reached(init_pose, goal_pose, 0.05):
        init_pose = rospy.wait_for_message('/id89/aruco_single/pose', PoseStamped)
        orientation_q = init_pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        Orientation = yaw
        dx = goal_pose.pose.position.x - init_pose.pose.position.x
        dy = goal_pose.pose.position.y - init_pose.pose.position.y
        distance = math.dist([init_pose.pose.position.x, init_pose.pose.position.y],
                             [goal_pose.pose.position.x, goal_pose.pose.position.y])
        goal_direct = math.atan2(dy, dx)

        if Orientation < 0:
            Orientation += 2 * math.pi
        if goal_direct < 0:
            goal_direct += 2 * math.pi

        theta = goal_direct - Orientation

        if theta < 0 and abs(theta) > abs(theta + 2 * math.pi):
            theta += 2 * math.pi
        elif theta > 0 and abs(theta - 2 * math.pi) < theta:
            theta -= 2 * math.pi

        plt.plot(init_pose.pose.position.x, init_pose.pose.position.y, marker='o')

        # PID control for linear and angular velocities
        linear_error = distance
        angular_error = theta
        linear_vel = pid_linear.compute(linear_error)
        angular_vel = pid_angular.compute(angular_error)

        twist.linear.x = linear_vel
        twist.angular.z = angular_vel
        cmd_pub.publish(twist)

        rate.sleep()

def check_goal_reached(init_pose, goal_pose, bias):
    if (init_pose.pose.position.x > goal_pose.pose.position.x - bias and
            init_pose.pose.position.x < goal_pose.pose.position.x + bias and
            init_pose.pose.position.y > goal_pose.pose.position.y - bias and
            init_pose.pose.position.y < goal_pose.pose.position.y + bias):
        return True
    else:
        return False

cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
init_pose = rospy.wait_for_message('/id89/aruco_single/pose', PoseStamped)
goal_pose1 = rospy.wait_for_message('/id88/aruco_single/pose', PoseStamped)
goal_pose2 = rospy.wait_for_message('/id87/aruco_single/pose', PoseStamped)
goal_pose = [goal_pose1, goal_pose2]

# Tune these PID parameters according to your robot dynamics
pid_linear = PIDController(kp=0.5, ki=0.1, kd=0.01)
pid_angular = PIDController(kp=1.0, ki=0.2, kd=0.05)

twist = Twist()
plt.figure(figsize=(8, 6))
plt.title("robot trajectory")
plt.xlabel("X")
plt.ylabel("Y")
plt.grid(True)

move(init_pose, goal_pose[0], pid_linear, pid_angular)
move(init_pose, goal_pose[1], pid_linear, pid_angular)

plt.show()
