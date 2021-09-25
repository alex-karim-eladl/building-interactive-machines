#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
# add any other python modules that you need here ...

# define joint positions per pose
poses_list = [
   {'joint_1': 0, 'joint_2': 0, 'joint_3': 0, 'joint_4': 0},
   {'joint_1': 0, 'joint_2': -0.95, 'joint_3': 0.41, 'joint_4': -1.0},
   {'joint_1': 0, 'joint_2': -1.45, 'joint_3': 1.45, 'joint_4': -1.53}
]

# poses_list = [[0,0,0,0], [0,-0.95,0.41,-1], [0,-1.45,1.45,-1.53]]

def pose_switcher():
   # Define publishers and init your node here.
   # Add a loop to request the robot to move its joints according to the
   # desired poses.
   pub1 = rospy.Publisher('joint_1/command', Float64, queue_size=10)
   pub2 = rospy.Publisher('joint_2/command', Float64, queue_size=10)
   pub3 = rospy.Publisher('joint_3/command', Float64, queue_size=10)
   pub4 = rospy.Publisher('joint_4/command', Float64, queue_size=10)

   rospy.init_node('pose_controller', anonymous=True)

   rate = rospy.Rate(0.2) # 10hz
   i = 0

   while not rospy.is_shutdown():
        if i == 2:
            i = 0
        else:
            i += 1

        joint1 = poses_list[i]['joint_1']
        joint2 = poses_list[i]['joint_2']
        joint3 = poses_list[i]['joint_3']
        joint4 = poses_list[i]['joint_4']

        rospy.loginfo(joint1)
        rospy.loginfo(joint2)
        rospy.loginfo(joint3)
        rospy.loginfo(joint4)

        pub1.publish(joint1)
        pub2.publish(joint2)
        pub3.publish(joint3)
        pub4.publish(joint4)
        rate.sleep()

if __name__ == '__main__':
   try:
       pose_switcher()
   except rospy.ROSInterruptException:
       pass
