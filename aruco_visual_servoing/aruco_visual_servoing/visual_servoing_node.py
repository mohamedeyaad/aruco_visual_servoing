import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
import cv2
import numpy as np

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from ros2_aruco_interfaces.msg import ArucoMarkers

class SequentialArucoChaser(Node):
    def __init__(self):
        super().__init__('sequential_aruco_chaser')

        # --- Configuration ---
        self.total_markers_expected = 5
        self.rotation_speed = 0.4
        self.center_kp = 0.4        # Proportional gain for centering (turn left/right)
        self.distance_kp = 0.25     # Proportional gain for approach (forward/back)
        self.target_dist = 1.0      # Stop 1.0 meter away from marker
        self.visit_duration = 1.0   # How long to stay at a marker before moving to next

        # --- State Variables ---
        self.found_ids = set()      # Stores all unique IDs seen during search
        self.target_list = []       # Will hold the sorted list of IDs to visit
        self.current_target_idx = 0 # Index of the marker we are currently chasing
        self.current_target_id = None
        
        # States: SEARCHING -> ALIGNING -> VISITING -> (Loop) -> COMPLETED
        self.state = "SEARCHING" 
        self.visit_start_time = None # Timer for the pause
        self.current_image = None

        # --- ROS Interface ---
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.img_pub = self.create_publisher(Image, '/aruco_target_circled', 10)

        self.create_subscription(ArucoMarkers, '/aruco_markers', self.aruco_callback, 10)
        self.create_subscription(Image, '/camera/image_raw', self.image_callback, qos_profile_sensor_data)

        self.bridge = CvBridge()
        self.get_logger().info("Sequential Chaser Started. Rotating to build map...")

    def image_callback(self, msg):
        """Cache image and draw circle only when we are in VISITING state."""
        try:
            self.current_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # We only publish the circled image when we have successfully reached the marker
            if self.state == "VISITING" and self.current_target_id is not None:
                self.process_and_publish_image()
                
        except Exception as e:
            self.get_logger().error(f"Image error: {e}")

    def aruco_callback(self, msg):
        twist = Twist()

        # Update our database of known markers
        for mid in msg.marker_ids:
            self.found_ids.add(mid)

        # --- STATE MACHINE ---

        # 1. SEARCHING: Spin until we know about all 5 markers
        if self.state == "SEARCHING":
            if len(self.found_ids) >= self.total_markers_expected:
                # Search complete. Sort IDs and pick the first one.
                self.target_list = sorted(list(self.found_ids))
                self.current_target_idx = 0
                self.current_target_id = self.target_list[0]
                
                self.get_logger().info(f"Map Complete! Sequence: {self.target_list}")
                self.get_logger().info(f"Targeting first marker: ID {self.current_target_id}")
                self.state = "ALIGNING"
                twist.angular.z = 0.0
            else:
                self.get_logger().info(f"Scanning... Found {len(self.found_ids)}/{self.total_markers_expected}", throttle_duration_sec=1)
                twist.angular.z = self.rotation_speed

        # 2. ALIGNING: Find specific target, center it, and approach it
        elif self.state == "ALIGNING":
            if self.current_target_id in msg.marker_ids:
                # Target is in view
                idx = msg.marker_ids.index(self.current_target_id)
                pose = msg.poses[idx]

                # Control Logic
                # 
                # X error = Horizontal offset
                # Z error = Distance from camera
                
                err_x = pose.position.x
                dist = pose.position.z

                # Rotate to center x
                twist.angular.z = -1.0 * err_x * self.center_kp

                # Move forward until distance is 1.0m
                if dist > self.target_dist:
                    twist.linear.x = (dist - self.target_dist) * self.distance_kp
                else:
                    # We are close enough. Check if we are centered enough.
                    if abs(err_x) < 0.05:
                        self.get_logger().info(f"Reached Marker {self.current_target_id}. Pausing.")
                        self.state = "VISITING"
                        self.visit_start_time = self.get_clock().now()
                        twist.linear.x = 0.0
                        twist.angular.z = 0.0

            else:
                # Target not in view? Rotate to find it.
                # (This handles the case where Marker 2 is behind Marker 1)
                twist.angular.z = self.rotation_speed

        # 3. VISITING: Stay still, publish circled image, wait, then pick next target
        elif self.state == "VISITING":
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            
            # Check how long we've been here
            now = self.get_clock().now()
            elapsed = (now - self.visit_start_time).nanoseconds / 1e9

            if elapsed > self.visit_duration:
                # Time's up, select next marker
                self.current_target_idx += 1
                
                if self.current_target_idx < len(self.target_list):
                    self.current_target_id = self.target_list[self.current_target_idx]
                    self.get_logger().info(f"Moving to next target: ID {self.current_target_id}")
                    self.state = "ALIGNING"
                else:
                    self.get_logger().info("All markers visited! Sequence complete.")
                    self.state = "COMPLETED"

        # 4. COMPLETED: Do nothing
        elif self.state == "COMPLETED":
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        self.cmd_pub.publish(twist)

    def process_and_publish_image(self):
        """Draws a circle around the CURRENT target ID."""
        if self.current_image is None: return

        draw_img = self.current_image.copy()
        
        # Detect again purely for 2D drawing coordinates
        aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_ARUCO_ORIGINAL)
        params = cv2.aruco.DetectorParameters_create()
        corners, ids, _ = cv2.aruco.detectMarkers(draw_img, aruco_dict, parameters=params)

        if ids is not None:
            for i, mk_id in enumerate(ids):
                # Only draw the circle for the target we are currently visiting
                if mk_id[0] == self.current_target_id:
                    c = corners[i][0]
                    # Calculate centroid
                    cx = int((c[0][0] + c[2][0]) / 2)
                    cy = int((c[0][1] + c[2][1]) / 2)
                    
                    # Draw Green Circle and Text
                    cv2.circle(draw_img, (cx, cy), 80, (0, 255, 0), 3)
                    cv2.putText(draw_img, f"ID: {self.current_target_id}", (cx - 20, cy - 60),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
     
        out_msg = self.bridge.cv2_to_imgmsg(draw_img, encoding='bgr8')
        self.img_pub.publish(out_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SequentialArucoChaser()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()