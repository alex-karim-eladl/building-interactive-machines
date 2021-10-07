#!/usr/bin/env python3
import math
import rospy
import numpy as np
import tf2_ros
import tf2_geometry_msgs
import geometry_msgs
from geometry_msgs.msg import PoseStamped
from shutter_lookat.msg import Target

class Node():
    def __init__(self): # listen for change in pose 
        self.tfbuffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tfbuffer)
        self.count = 0
        rospy.Subscriber('/target', Target, self.callback)
        rospy.spin()

    def get_transform(self, pose):
        rate = rospy.Rate(1000)
        while not rospy.is_shutdown():
            try:
                trans = self.tfbuffer.lookup_transform('camera_color_optical_frame', 'base_footprint', pose.header.stamp, rospy.Duration(0.5))
                return trans
            except (tf2_ros.ConnectivityException):
                rate.sleep()
                continue


    def callback(self, data):        
        trans = self.get_transform(data.pose)
        self.count += 1
        # transfrom pose from base -> cam_color_opt
        pose_trans = tf2_geometry_msgs.do_transform_pose(data.pose, trans)

        # transfrom cam_color_opt -> target
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = data.pose.header.stamp
        t.header.frame_id = 'camera_color_optical_frame'
        t.child_frame_id = 'target'  

        t.transform.translation.x = pose_trans.pose.position.x
        t.transform.translation.y = pose_trans.pose.position.y
        t.transform.translation.z = pose_trans.pose.position.z
        # t.transform.translation.x = -data.pose.pose.position.y + trans.transform.translation.y
        # t.transform.translation.y = -data.pose.pose.position.z + trans.transform.translation.z
        # t.transform.translation.z = data.pose.pose.position.x - trans.transform.translation.x
        
        t.transform.rotation.x = 0
        t.transform.rotation.y = 0
        t.transform.rotation.z = 0
        t.transform.rotation.w = 1

        br = tf2_ros.TransformBroadcaster()
        br.sendTransform(t)



if __name__ == '__main__':
    rospy.init_node('publish_target_relative_to_realsense_camera', anonymous=True)

    try:
        n = Node()
    except rospy.ROSInterruptException: 
        pass