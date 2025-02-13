#!/usr/bin/env python3
import rospy
import math
import random
import time
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

# Global variables for current robot pose and list of garbage targets
robot_pose = None
garbage_list = []  # Stores incoming PoseStamped messages from the spawn node

class PID:
    def __init__(self, Kp, Ki, Kd, setpoint=0.0, output_limits=(None, None)):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self._integral = 0.0
        self._prev_error = None
        self.output_limits = output_limits

    def update(self, measurement, dt):
        error = measurement - self.setpoint
        P = self.Kp * error
        self._integral += error * dt
        I = self.Ki * self._integral
        D = 0.0
        if self._prev_error is not None and dt > 0:
            D = self.Kd * (error - self._prev_error) / dt
        self._prev_error = error
        output = P + I + D
        lower, upper = self.output_limits
        if lower is not None:
            output = max(lower, output)
        if upper is not None:
            output = min(upper, output)
        return output

def odom_callback(msg):
    global robot_pose
    robot_pose = msg.pose.pose

def garbage_callback(msg):
    global garbage_list
    # Simply add every new garbage position to the list.
    garbage_list.append(msg)

def get_yaw_from_quaternion(orientation):
    quat = (orientation.x, orientation.y, orientation.z, orientation.w)
    _, _, yaw = euler_from_quaternion(quat)
    return yaw

def transform_error_to_robot_frame(goal_x, goal_y, robot_pose):
    """
    Transforms the vector difference (goal - robot) from the world frame
    to the robot's frame.
    Returns (error_x, error_y) where error_x is the forward/backward error.
    """
    dx = goal_x - robot_pose.position.x
    dy = goal_y - robot_pose.position.y
    robot_yaw = get_yaw_from_quaternion(robot_pose.orientation)
    # Rotate the world vector by -robot_yaw to obtain robot-frame coordinates
    error_x = dx * math.cos(robot_yaw) + dy * math.sin(robot_yaw)
    error_y = -dx * math.sin(robot_yaw) + dy * math.cos(robot_yaw)
    return error_x, error_y

def navigate_to_garbage():
    global robot_pose, garbage_list

    rospy.init_node('navigate_to_garbage')
    rospy.Subscriber('/odom', Odometry, odom_callback)
    rospy.Subscriber('/garbage_positions', PoseStamped, garbage_callback)
    cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    rate = rospy.Rate(10)  # 10 Hz loop rate
    last_time = rospy.Time.now().to_sec()

    # PID controller for linear velocity in the robot's x-direction.
    # The output is clamped to a range that suits your robot (here, between -0.5 and 0.5 m/s).
    pid_linear = PID(Kp=0.8, Ki=0.0, Kd=0.1, setpoint=0.0, output_limits=(-0.5, 0.5))
    
    # Desired threshold for the error in the robot frame (in meters).
    error_threshold = 0.05

    while not rospy.is_shutdown():
        if robot_pose is None or not garbage_list:
            rate.sleep()
            continue

        current_time = rospy.Time.now().to_sec()
        dt = current_time - last_time
        last_time = current_time

        # Select the nearest garbage target based on Euclidean distance in the world frame.
        current_robot_x = robot_pose.position.x
        current_robot_y = robot_pose.position.y
        min_dist = float('inf')
        target_garbage = None
        for g in garbage_list:
            dx = g.pose.position.x - current_robot_x
            dy = g.pose.position.y - current_robot_y
            dist = math.sqrt(dx**2 + dy**2)
            if dist < min_dist:
                min_dist = dist
                target_garbage = g

        if target_garbage is None:
            rate.sleep()
            continue

        # Extract garbage coordinates (world frame)
        Gx = target_garbage.pose.position.x
        Gy = target_garbage.pose.position.y

        # Compute the vector from robot to garbage in world frame and its angle.
        dx = Gx - current_robot_x
        dy = Gy - current_robot_y
        theta_v = math.atan2(dy, dx)

        # Choose a lateral offset L of ±0.5 m randomly.
        L = 0.5 if random.choice([True, False]) else -0.5

        # Compute a goal position (in world frame) so that the garbage ends up at a lateral offset of L.
        # This is done by shifting the garbage position along the direction perpendicular to the line from robot to garbage.
        goal_x = Gx - L * math.cos(theta_v + math.pi/2)
        goal_y = Gy - L * math.sin(theta_v + math.pi/2)

        # Transform the goal error into the robot's frame.
        error_x, error_y = transform_error_to_robot_frame(goal_x, goal_y, robot_pose)
        
        # For x-direction only, we use error_x.
        linear_speed = pid_linear.update(error_x, dt)

        twist = Twist()
        twist.linear.x = linear_speed
        twist.angular.z = 0.0  # No turning commands since the robot can only go forward/backward.

        rospy.loginfo("Error X (robot frame): {:.3f}, Commanded speed: {:.3f}".format(error_x, linear_speed))

        # Check if the error is within our threshold.
        if abs(error_x) < error_threshold:
            twist.linear.x = 0.0
            cmd_pub.publish(twist)
            rospy.loginfo("Reached target: garbage lateral offset reached (L = {:.2f} m)".format(L))
            garbage_list.remove(target_garbage)
            # Reset the PID controller for the next target.
            pid_linear._integral = 0.0
            pid_linear._prev_error = None
            time.sleep(1)  # brief pause before processing the next target
        else:
            cmd_pub.publish(twist)
            
        rate.sleep()

if __name__ == '__main__':
    try:
        navigate_to_garbage()
    except rospy.ROSInterruptException:
        pass
