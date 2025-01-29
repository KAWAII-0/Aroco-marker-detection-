



from cv_bridge import CvBridge  
import cv2  
import cv2.aruco as aruco  
import rclpy  
from geometry_msgs.msg import Twist  
from rclpy.node import Node  
from sensor_msgs.msg import Image  

class VisionNode(Node):
    def __init__(self):
        super().__init__('robot_vision_node')

        #  CvBridge to convert ROS images to OpenCV format
        self.bridge_converter = CvBridge()

        # Subscribe to the 'image_raw' topic 
        self.image_subscriber = self.create_subscription(
            Image,
            'image_raw',
            self.process_image_data,
            10
        )

        # Publisher for sending movement commands to control the robot
        self.command_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info("Vision Node started, subscribing to 'image_raw'.")

        
        self.marker_dictionary = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        self.detection_params = aruco.DetectorParameters()

    def generate_motion_command(self, forward_speed, lateral_speed, turn_speed):
        """
        A helper function to create a movement command (Twist) for the robot.
        """
        command = Twist()
        command.linear.x = forward_speed
        #command.linear.y = lateral_speed
        command.angular.z = turn_speed
        return command

    def process_image_data(self, incoming_image_msg):
        """
        Callback function to process incoming image data from the camera.
        """
        try:
            # Convert the incoming ROS image to an OpenCV image (in BGR8 format)
            opencv_image = self.bridge_converter.imgmsg_to_cv2(incoming_image_msg, "bgr8")

            # Get image dimensions 
            img_height, img_width, _ = opencv_image.shape

            # Convert the image to grayscale 
            grayscale_image = cv2.cvtColor(opencv_image, cv2.COLOR_BGR2GRAY)

            # Detect ArUco markers in the grayscale image
            detected_corners, detected_ids, _ = aruco.detectMarkers(
                grayscale_image,
                self.marker_dictionary,
                parameters=self.detection_params
            )

            # process the location of the marker 
            if detected_ids is not None:
                # Draw detected markers on the image for visualization
                aruco.drawDetectedMarkers(opencv_image, detected_corners, detected_ids)
                self.get_logger().info(f"Detected marker IDs: {detected_ids.flatten().tolist()}")

                for marker_id, corners in zip(detected_ids.flatten(), detected_corners):
                    # Calculate the marker's center
                    marker_center = corners[0].mean(axis=0)
                    marker_center_x, marker_center_y = marker_center
                    image_center_x, image_center_y = img_width / 2, img_height / 2

                    # motion variables
                    forward_velocity = 0.0
                    lateral_velocity = 0.0
                    angular_velocity = 0.0

                    # Logic for forward/backward motion 
                    if marker_center_y < image_center_y - 20:  # Marker is above the center
                        forward_velocity = 2.0  # Adjust forward speed
                        self.get_logger().info(f"Marker {marker_id}: Above center. Moving forward.")
                    elif marker_center_y > image_center_y + 20:  # Marker is below the center
                        forward_velocity = -2.0  # Adjust backward speed
                        self.get_logger().info(f"Marker {marker_id}: Below center. Moving backward.")

                    # Logic for lateral movement with angular adjustments (based on the horizontal position of the marker)
                    if marker_center_x < image_center_x - 10:  # Marker is to the left
                        lateral_velocity = 2.0  # Move to the left
                        angular_velocity = 1.0  # Rotate to the left
                        self.get_logger().info(f"Marker {marker_id}: Left of center. Moving left.")
                    elif marker_center_x > image_center_x + 10:  # Marker is to the right
                        lateral_velocity = -2.0  # Move to the right
                        angular_velocity = -1.0  # Rotate to the right
                        self.get_logger().info(f"Marker {marker_id}: Right of center. Moving right.")

                    # Publish commands to control the robot
                    motion_command = self.generate_motion_command(forward_velocity, lateral_velocity, angular_velocity)
                    self.command_publisher.publish(motion_command)

            else:
                motion_command = self.generate_motion_command(0,0,0)
                self.command_publisher.publish(motion_command)

            # Display the processed image with markers
            cv2.imshow("ArUco Marker Detection", opencv_image)
            cv2.waitKey(1)

        except Exception as error:
            self.get_logger().error(f"Error in process_image_data: {error}")

def main(args=None):
    """
    Main function to initialize and run the node.
    """
    rclpy.init(args=args)

    # Create and run the VisionNode instance
    vision_node = VisionNode()

    try:
        # Keep the node running
        rclpy.spin(vision_node)
    except KeyboardInterrupt:
        # Handle manual interruption (Ctrl+C)
        pass
    finally:
        # Ensure proper cleanup of resources
        vision_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
