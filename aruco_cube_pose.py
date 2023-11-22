#!/usr/bin/env python

import numpy as np
import cv2
import cv2.aruco as aruco
import rospy
import tf

from visualization_msgs.msg import Marker
from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseStamped, Point, Quaternion


class ArUcoDetector:
    def __init__(self):
        rospy.init_node('aruco_detector', anonymous=True)

        # Subscribe to camera info and image topics
        self.camera_info_sub = rospy.Subscriber('/camera/color/camera_info', CameraInfo, self.camera_info_callback)
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)

        # Initialize camera matrix and distortion coefficients
        self.camera_matrix = None
        self.dist_coeffs = None

        # Initialize OpenCV bridge
        self.bridge = CvBridge()

        # Define the ArUco dictionary
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_1000)

        # Publisher for cube pose
        self.cube_pose_pub = rospy.Publisher('/aruco_cube/pose', PoseStamped, queue_size=10)

        # TF Broadcaster for coordinate transformation
        self.tf_broadcaster = tf.TransformBroadcaster()

        # Define the marker points for each face of the cube
        self.offset = 0.005 # difference between marker size and cube size / 2
        self.marker_size = 0.065 + self.offset  # Marker size: 5 cm      
        c_pt = self.marker_size / 2
        # Define the ArUco board parameters
        board_ids = np.array([[0], [1], [2], [3], [4], [5]], dtype=np.int32)
        board_corners = [
                        np.array([[-c_pt, c_pt, c_pt], [c_pt, c_pt, c_pt], [c_pt, -c_pt, c_pt], [-c_pt, -c_pt, c_pt]], dtype=np.float32),
                        np.array([[-c_pt, -c_pt, c_pt], [c_pt, -c_pt, c_pt], [c_pt, -c_pt, -c_pt], [-c_pt, -c_pt, -c_pt]], dtype=np.float32),
                        np.array([[-c_pt, c_pt, c_pt], [-c_pt, -c_pt, c_pt], [-c_pt, -c_pt, -c_pt], [-c_pt, c_pt, -c_pt]], dtype=np.float32),
                        np.array([[c_pt, c_pt, c_pt], [-c_pt, c_pt, c_pt], [-c_pt, c_pt, -c_pt], [c_pt, c_pt, -c_pt]], dtype=np.float32),
                        np.array([[c_pt, -c_pt, c_pt], [c_pt, c_pt, c_pt], [c_pt, c_pt, -c_pt], [c_pt, -c_pt, -c_pt]], dtype=np.float32),
                        np.array([[-c_pt, -c_pt, -c_pt], [c_pt, -c_pt, -c_pt], [c_pt, c_pt, -c_pt], [-c_pt, c_pt, -c_pt]], dtype=np.float32)
                        ]

        # Create the ArUco board
        self.board = aruco.Board_create(
            board_corners,
            self.aruco_dict,
            board_ids
        )

        self.marker_pub = rospy.Publisher("/visualization_marker_real_cube", Marker, queue_size = 10)
        self.marker = Marker()

    def create_marker(self, marker, marker_pub, type=2, target_location=None, color=None, scale=None):
        marker.header.frame_id = "camera_color_optical_frame"
        marker.header.stamp = rospy.Time.now()

        # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
        marker.type = type
        marker.id = 0

        # Set the scale of the marker
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale

        # Set the color
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = 1.0

        # Set the pose of the marker
        marker.pose.position = target_location.pose.position
        marker.pose.orientation = target_location.pose.orientation
        marker_pub.publish(marker)

    def camera_info_callback(self, msg):
        # Extract camera matrix and distortion coefficients from CameraInfo message
        self.camera_matrix = np.array(msg.K).reshape((3, 3))
        self.dist_coeffs = np.array(msg.D)

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError as e:
            print(e)
            return

        if self.camera_matrix is not None and self.dist_coeffs is not None:
            # Convert the image to grayscale
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

            # Detect ArUco markers
            corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict)

            # Pose estimation using the ArUco board
            _, rvec, tvec = aruco.estimatePoseBoard(corners, ids, self.board, self.camera_matrix, self.dist_coeffs, np.empty(1), np.empty(1))

            # Draw markers on the image for visualization
            cv_image = aruco.drawAxis(cv_image, self.camera_matrix, self.dist_coeffs, rvec, tvec, self.marker_size)
            cv_image = aruco.drawDetectedMarkers(cv_image, corners, ids)

            # Extract translation values from the pose
            translation = tvec.flatten() 
            translation[2] -= 0.1 # adjustment for basse_link to aruco_cube pose (TODO recaliberate)

            # Extract quaternion values from the pose
            rotation = rvec.ravel()
            quaternion = tf.transformations.quaternion_from_euler(rotation[0], rotation[1], rotation[2])


            # Publish the pose as a PointStamped message
            cube_pose_msg = PoseStamped()
            cube_pose_msg.header.frame_id = 'camera_color_optical_frame'
            cube_pose_msg.header.stamp = rospy.Time.now()
            cube_pose_msg.pose.position = Point(x=translation[0], y=translation[1], z=translation[2])
            cube_pose_msg.pose.orientation = Quaternion(x=quaternion[0], y=quaternion[1], z=quaternion[2], w=quaternion[3])
            self.cube_pose_pub.publish(cube_pose_msg)

            self.create_marker(self.marker, self.marker_pub, type=1, target_location=cube_pose_msg, color=[1, 0, 0], scale=0.075) # create marker for rviz visualization

            # Display the image with markers and poses
            cv2.imshow("ArUco Detection", cv_image)
            cv2.waitKey(1)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    aruco_detector = ArUcoDetector()
    aruco_detector.run()
