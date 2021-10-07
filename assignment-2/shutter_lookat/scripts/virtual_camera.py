#!/usr/bin/env python3
import cv2
import numpy as np
import rospy
import tf2_ros
import tf2_geometry_msgs
from cv_bridge import CvBridge, CvBridgeError
from shutter_lookat.msg import Target
from sensor_msgs.msg import Image, CameraInfo
import geometry_msgs
from geometry_msgs.msg import PoseStamped


def project_3D_point(x, y, z, K):
    """
    Project 3D point [x, y, z]^T in camera frame onto the camera image using the intrinsic parameters K
    :param x: x coordinate of the 3D point (in the camera frame)
    :param y: y coordinate of the 3D point (in the camera frame)
    :param z: z coordinate of the 3D point (in the camera frame)
    :param K: 3x3 numpy array with intrinsic camera parameters
    :return: tuple with 2D coordinates of the projected 3D point on the camera plane
    """
    X = np.array([x,y,z,1]).reshape(4,1)
    E = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0]])
    P = np.dot(K, E)
    image_coord = np.dot(P,X)
    
    x_pos = int(image_coord[0][0])
    y_pos = int(image_coord[1][0])
    # print(x_pos, y_pos)
    # change this pass for an appropriate return statement, e.g., return pixel_x, pixel_y
    return x_pos, y_pos


def draw_image(x, y, z, K, width, height, **kwargs):
    """
    Project the target and create a virtual camera image
    :param x: x coordinate of the 3D point (in the camera frame)
    :param y: y coordinate of the 3D point (in the camera frame)
    :param z: z coordinate of the 3D point (in the camera frame)
    :param K: 3x3 numpy array with intrinsic camera parameters
    :param width: desired image width
    :param height: desired image height
    :param kwargs: extra parameters for the function (used in Part IV of the assignment)
    :return: color image as a numpy array. The image should have dimensions height x width x 3 and be of type uint8.
    """

    # NOTE: You should project the position of the moving object on the camera image using
    # the project_3D_point() function. You need to complete it as part of this assignment.

    # create image
    image = np.zeros((height, width, 3), np.uint8) # width and height are the dimensions of the image
    # color image
    image[:,:] = (255,255,255) # (B, G, R)

    # print(x,y,z)
    # print(x, y, z)

    if (z <= 0):
        rospy.logwarn("Warning: Target is behind the camera (z={})".format(z))
        return image
    # rospy.logwarn("(z={})".format(z))
    x_pos, y_pos = project_3D_point(x,y,z,K)

    cv2.circle(image, (x_pos,y_pos), 12, (0,0,255), 3) 
    
    # change this pass for an appropriate return statement, e.g., return image (where image is a numpy array)
    return image


class VirtualCameraNode:
    """
    Node for Part III for the assignment 1. Renders an image from a virtual camera.
    """

    def __init__(self):
        """
        Constructor
        """
        self.count = 0
        # Init the node
        rospy.init_node('virtual_camera')

        self.tfBuffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tfBuffer)


        # Complete this constructor with instance variables that you need for your code and any
        # additional publishers or subscribers...

        cx=320       # x-coordinate of principal point in terms of pixel dimensions
        cy=240       # y-coordinate of principal point in terms of pixel dimensions
        fx=349       # focal length in terms of pixel dimensions in the x direction
        fy=349       # focal length in terms of pixel dimensions in the y direction
        s = 0        # note: assume there's no skew.

        self.K = np.array([[fx,s,cx],[0,fy,cy],[0,0,1]])

        # finally, start receiving target messages in your node...
        rospy.Subscriber('target', Target, self.target_callback, queue_size=5)
        rospy.spin()


    def make_camera_info_message(self, stamp, frame_id, image_width, image_height, cx, cy, fx, fy):
        """
        Build CameraInfo message
        :param stamp: timestamp for the message
        :param frame_id: frame id of the camera
        :param image_width: image width
        :param image_height: image height
        :param cx: x-coordinate of principal point in terms of pixel dimensions
        :param cy: y-coordinate of principal point in terms of pixel dimensions
        :param fx: focal length in terms of pixel dimensions in the x direction
        :param fy: focal length in terms of pixel dimensions in the y direction
        :return: CameraInfo message with the camera calibration parameters.
        """
        camera_info_msg = CameraInfo()
        camera_info_msg.header.stamp = stamp
        camera_info_msg.header.frame_id = frame_id
        camera_info_msg.width = image_width
        camera_info_msg.height = image_height
        camera_info_msg.K = [fx, 0, cx, 0, fy, cy, 0, 0, 1]
        camera_info_msg.D = [0, 0, 0, 0, 0]
        camera_info_msg.R = [1, 0, 0, 0, 1, 0, 0, 0, 1]
        camera_info_msg.P = [fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1, 0]
        camera_info_msg.distortion_model = "plumb_bob"
        return camera_info_msg


    def get_transform(self, pose):
        rate = rospy.Rate(1000)
        while not rospy.is_shutdown():
            try:
<<<<<<< HEAD
                trans = self.tfBuffer.lookup_transform('camera_color_optical_frame', 'base_footprint', pose.header.stamp, rospy.Duration(0.5))
=======
                trans = self.tfBuffer.lookup_transform('base_footprint', 'camera_color_optical_frame', pose.header.stamp, rospy.Duration(0.1))
>>>>>>> 1c905ad5959faf7234368ca1891f993520f66e1c
                return trans
            except (tf2_ros.ConnectivityException):
                rate.sleep()
                continue


    def target_callback(self, target_msg):
        """
        Target callback
        :param target_msg: target message
        """
<<<<<<< HEAD
        self.count += 1
=======

>>>>>>> 1c905ad5959faf7234368ca1891f993520f66e1c
        # Convert target message to "camera_color_optical_frame" frame to get the target's x,y,z coordinates
        # relative to the camera...
        t = self.get_transform(target_msg.pose)
       
<<<<<<< HEAD
        pose_trans = tf2_geometry_msgs.do_transform_pose(target_msg.pose, t)
        px = pose_trans.pose.position.x
        py = pose_trans.pose.position.y
        pz = pose_trans.pose.position.z

        # Draw the camera image. Use the draw_image(x, y, z, K, width, height) function to this end....
        # cv_image = draw_image(t.transform.translation.x,t.transform.translation.y,t.transform.translation.z, self.K, 640, 480)
=======
        target_cam_frame = tf2_geometry_msgs.do_transform_pose(target_msg.pose, t)
        px = - target_msg.pose.pose.position.y + t.transform.translation.y
        py = - target_msg.pose.pose.position.z + t.transform.translation.z
        pz = target_msg.pose.pose.position.x - t.transform.translation.x
        
    
        # Draw the camera image. Use the draw_image(x, y, z, K, width, height) function to this end....
>>>>>>> 1c905ad5959faf7234368ca1891f993520f66e1c
        cv_image = draw_image(px,py,pz, self.K, 640, 480)
        image_width = cv_image.shape[1]
        image_height = cv_image.shape[0]
    
        # # Publish the resulting image....
        bridge = CvBridge()
        image_msg = bridge.cv2_to_imgmsg(cv_image, encoding="passthrough")
        # print(image_msg.header)
        image_msg.header.frame_id = 'camera_color_optical_frame'
        image_msg.header.stamp = rospy.Time.now()
        # print('pub')
        image_pub = rospy.Publisher('/virtual_camera/image_raw', Image, queue_size=10)
        camerainfo_pub = rospy.Publisher('/virtual_camera/camera_info', CameraInfo, queue_size=10)

        image_pub.publish(image_msg)

        cx=320       # x-coordinate of principal point in terms of pixel dimensions
        cy=240       # y-coordinate of principal point in terms of pixel dimensions
        fx=349       # focal length in terms of pixel dimensions in the x direction
        fy=349       # focal length in terms of pixel dimensions in the y direction
        
        camerainfo_msg = self.make_camera_info_message(image_msg.header.stamp, image_msg.header.frame_id, image_width, image_height,cx, cy,fx, fy)
        camerainfo_pub.publish(camerainfo_msg)        
        


if __name__ == '__main__':
    try:
        node = VirtualCameraNode()
    except rospy.ROSInterruptException:
        pass
