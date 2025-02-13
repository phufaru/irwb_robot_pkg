#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState

def joint_state_callback(msg):
    rospy.loginfo("Current joint states:")
    for i in range(len(msg.name)):
        rospy.loginfo(f"{msg.name[i]}: {msg.position[i]:.2f} rad")

def monitor_joint_states():
    rospy.init_node('joint_state_monitor', anonymous=True)
    rospy.Subscriber('/joint_states', JointState, joint_state_callback)
    rospy.spin()

if __name__ == '__main__':
    monitor_joint_states()