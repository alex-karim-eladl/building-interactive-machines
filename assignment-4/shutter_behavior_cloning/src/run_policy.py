#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64
from urdf_parser_py.urdf import URDF
import tensorflow as tf

def normalize_data_per_row(data, mean, stdev):
    """
    Normalize a give matrix of data (samples must be organized per row)
    :param data: input data
    :param mean: mean for normalization
    :param stdev: standard deviation for normalization
    :return: whitened data, (data - mean) / stdev
    """

    # sanity checks!
    assert len(data.shape) == 2, "Expected the input data to be a 2D matrix"
    assert data.shape[1] == mean.shape[1], "Data - Mean size mismatch ({} vs {})".format(data.shape[1], mean.shape[1])
    assert data.shape[1] == stdev.shape[1], "Data - StDev size mismatch ({} vs {})".format(data.shape[1], stdev.shape[1])

    centered = data - np.tile(mean, (data.shape[0], 1))
    normalized_data = np.divide(centered, np.tile(stdev, (data.shape[0],1)))

    return normalized_data

def load_param(input_file):

    mean = np.zeros((5,1))
    stdev = np.zeros((5,1))

    with open(input_file, 'r') as fid:
        for line in fid:
            if line[0] == "#":
                # skip comments
                continue

            line = line.rstrip().split(" ")
            # print(line)
            assert len(line) == 10, "Expected each line to have 8 elements."

            mean = np.column_stack(([float(line[0])], [float(line[2])], [float(line[4])], [float(line[6])], [float(line[8])]))
            stdev = np.column_stack(([float(line[1])], [float(line[3])], [float(line[5])], [float(line[7])], [float(line[9])]))

    return mean, stdev


class RunPolicyNode(object):
    """
    Node that controls the robot to make it point towards a target.
    """

    def __init__(self):
        rospy.init_node('expert_opt')

        # params
        self.base_link = rospy.get_param("~base_link", "base_link")
        self.biceps_link = rospy.get_param("~biceps_link", "biceps_link")
        self.camera_link = rospy.get_param("~camera_link", "camera_color_optical_frame")

        self.model_file = rospy.get_param("~model")              # required path to model file
        self.normp_file = rospy.get_param("~norm_params","")    # optional path to normalization parameters (empty str means no norm params)

        self.mean, self.stdev = load_param(self.normp_file)

        # TODO - complete the line below to load up your model and create any necessary class instance variables
        self.model = tf.keras.models.load_model(self.model_file)

        # joint values
        self.joint1 = None
        self.joint3 = None

        # get robot model
        self.robot = URDF.from_parameter_server()

        # joint publishers
        self.joint1_pub = rospy.Publisher('/joint_1/command', Float64, queue_size=5)    # command for base rotation
        self.joint3_pub = rospy.Publisher('/joint_3/command', Float64, queue_size=5)    # command for robot tilt

        # joint subscriber
        rospy.Subscriber('/joint_states', JointState, self.joints_callback, queue_size=5)
        rospy.Subscriber('/target', PoseStamped, self.target_callback, queue_size=5)

        rospy.spin()


    def joints_callback(self, msg):
        """
        Joints callback
        :param msg: joint state
        """
        joint1_idx = -1
        joint3_idx = -1
        for i in range(len(msg.name)):
            if msg.name[i] == 'joint_1':
                joint1_idx = i
            elif msg.name[i] == 'joint_3':
                joint3_idx = i
        assert joint1_idx >= 0 and joint3_idx > 0, \
            "Missing joints from joint state! joint1 = {}, joint3 = {}".format(joint1_idx, joint3_idx)
        self.joint1 = msg.position[joint1_idx]
        self.joint3 = msg.position[joint3_idx]

    def compute_joints_position(self, msg):
        """
        Helper function to compute the required motion to make the robot's camera look towards the target
        :param msg: target message that was received by the target callback
        :return: tuple with new joint positions for joint1 and joint3, or None if the computation failed
        """

        # TODO - remove None return statement and complete function with the logic that runs your model to compute new
        # joint positions 1 & 3 for the robot...
        input = np.array([[msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, self.joint1, self.joint3]])
        # print(self.stdev.shape)
        norm_input = normalize_data_per_row(input, self.mean, self.stdev)
        predicted = self.model.predict(norm_input)

        return predicted

    def target_callback(self, msg):
        """
        Target callback
        :param msg: target message
        """
        # check that the data is consistent with our model and that we have current joint information...
        if msg.header.frame_id != "base_footprint":
            rospy.logerr("Expected the input target to be in the frame 'base_footprint' but got the {} frame instead. "
                         "Failed to command the robot".format(msg.header.frame_id))
            return

        if self.joint1 is None:
            rospy.logwarn("Joint 1 is unknown. Waiting to receive joint states.")
            return

        # compute the required motion to make the robot look towards the target
        joint_angles = self.compute_joints_position(msg)
        if joint_angles is None:
            # we are done. we did not get a solution
            rospy.logwarn("The compute_joints_position() function returned None. Failed to command the robot.")
            return
        else:
            # upack result
            # print(joint_angles)
            new_j1 = joint_angles[0,0]
            new_j3 = joint_angles[0,1]

        # publish command
        self.joint1_pub.publish(Float64(new_j1))
        self.joint3_pub.publish(Float64(new_j3))


if __name__ == '__main__':
    try:
        node = RunPolicyNode()
    except rospy.ROSInterruptException:
        pass
