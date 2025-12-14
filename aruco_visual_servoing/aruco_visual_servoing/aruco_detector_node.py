import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
import cv2
import numpy as np

from sensor_msgs.msg import CameraInfo, Image
from geometry_msgs.msg import Pose
from aruco_interfaces.msg import ArucoMarkers
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

class ArucoNode(Node):
    def __init__(self):
        super().__init__('aruco_node')

        # --- Parameters (Matching your YAML) ---
        self.declare_parameter("marker_size", 0.4, 
                               ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE))
        self.declare_parameter("aruco_dictionary_id", "DICT_ARUCO_ORIGINAL", 
                               ParameterDescriptor(type=ParameterType.PARAMETER_STRING))
        self.declare_parameter("image_topic", "/camera/image_raw", 
                               ParameterDescriptor(type=ParameterType.PARAMETER_STRING))
        self.declare_parameter("camera_info_topic", "/camera/camera_info", 
                               ParameterDescriptor(type=ParameterType.PARAMETER_STRING))

        # Fetch values
        self.marker_size = self.get_parameter("marker_size").value
        dictionary_id_name = self.get_parameter("aruco_dictionary_id").value
        image_topic = self.get_parameter("image_topic").value
        info_topic = self.get_parameter("camera_info_topic").value

        # --- Setup Aruco Dictionary ---
        # Map string names to OpenCV constants
        self.aruco_dict_map = {
            "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
            "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
            "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
            "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
            "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
            "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
            "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
            "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
            "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
            "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
            "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
            "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
            "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
            "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
            "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
            "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
            "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL
        }

        if dictionary_id_name not in self.aruco_dict_map:
            self.get_logger().error(f"Unknown dictionary: {dictionary_id_name}. Defaulting to 5X5_250")
            dictionary_id = cv2.aruco.DICT_ARUCO_ORIGINAL
        else:
            dictionary_id = self.aruco_dict_map[dictionary_id_name]

        self.aruco_dictionary = cv2.aruco.Dictionary_get(dictionary_id)
        self.aruco_parameters = cv2.aruco.DetectorParameters_create()

        # --- Variables ---
        self.camera_matrix = None
        self.dist_coeffs = None
        self.bridge = CvBridge()

        # --- Subscribers & Publishers ---
        self.info_sub = self.create_subscription(CameraInfo, info_topic, self.info_callback, 10)
        
        # Use sensor data QoS for image to handle high bandwidth/wifi drops better
        self.img_sub = self.create_subscription(Image, image_topic, self.image_callback, qos_profile_sensor_data)

        # The topic your Chaser is listening to
        self.marker_pub = self.create_publisher(ArucoMarkers, '/aruco_markers', 10)
        
        self.get_logger().info(f"Aruco Detector started. Looking for {dictionary_id_name}...")

    def info_callback(self, msg):
        """ Get camera intrinsics. Only needed once. """
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape((3, 3))
            self.dist_coeffs = np.array(msg.d)
            self.get_logger().info("Camera info received.")
            # We don't destroy subscription in case camera params change, 
            # but usually you could self.destroy_subscription(self.info_sub) here.

    def image_callback(self, msg):
        # We cannot estimate pose without camera info
        if self.camera_matrix is None:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Detect
            corners, marker_ids, rejected = cv2.aruco.detectMarkers(
                cv_image, self.aruco_dictionary, parameters=self.aruco_parameters
            )

            if marker_ids is not None:
                # Estimate Pose
                # rvecs: rotation vectors, tvecs: translation vectors
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corners, self.marker_size, self.camera_matrix, self.dist_coeffs
                )

                # Prepare Message
                markers_msg = ArucoMarkers()
                markers_msg.header.stamp = msg.header.stamp
                markers_msg.header.frame_id = msg.header.frame_id # Important: Output is in Camera Optical Frame

                for i in range(len(marker_ids)):
                    pose = Pose()
                    
                    # Position (tvec is [x, y, z])
                    pose.position.x = tvecs[i][0][0]
                    pose.position.y = tvecs[i][0][1]
                    pose.position.z = tvecs[i][0][2]

                    # Orientation (Convert Rodrigues vector to Quaternion)
                    # rvec is 1x1x3
                    rot_matrix, _ = cv2.Rodrigues(rvecs[i])
                    
                    # Manual rotation matrix to quaternion conversion
                    # (Standard method to avoid heavy external dependencies like scipy for just this)
                    q = self.rotation_matrix_to_quaternion(rot_matrix)
                    
                    pose.orientation.x = q[0]
                    pose.orientation.y = q[1]
                    pose.orientation.z = q[2]
                    pose.orientation.w = q[3]

                    markers_msg.poses.append(pose)
                    markers_msg.marker_ids.append(marker_ids[i][0])

                self.marker_pub.publish(markers_msg)

        except Exception as e:
            self.get_logger().error(f"Error processing image: {str(e)}")

    def rotation_matrix_to_quaternion(self, R):
        """
        Calculates quaternion (x, y, z, w) from 3x3 rotation matrix R.
        """
        trace = np.trace(R)
        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1.0)
            w = 0.25 / s
            x = (R[2, 1] - R[1, 2]) * s
            y = (R[0, 2] - R[2, 0]) * s
            z = (R[1, 0] - R[0, 1]) * s
        else:
            if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
                s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
                w = (R[2, 1] - R[1, 2]) / s
                x = 0.25 * s
                y = (R[0, 1] + R[1, 0]) / s
                z = (R[0, 2] + R[2, 0]) / s
            elif R[1, 1] > R[2, 2]:
                s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
                w = (R[0, 2] - R[2, 0]) / s
                x = (R[0, 1] + R[1, 0]) / s
                y = 0.25 * s
                z = (R[1, 2] + R[2, 1]) / s
            else:
                s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
                w = (R[1, 0] - R[0, 1]) / s
                x = (R[0, 2] + R[2, 0]) / s
                y = (R[1, 2] + R[2, 1]) / s
                z = 0.25 * s
        return [x, y, z, w]

def main(args=None):
    rclpy.init(args=args)
    node = ArucoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()