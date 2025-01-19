import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco

class ArUcoTracker(Node):
    def __init__(self):
        super().__init__('aruco_tracker')

        # Subscription to the image topic
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.process_image,
            10
        )

        # Publisher for controlling robot's velocity
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Adjust the velocity scale factor
        self.velocity_scale = 0.3

        # Initialize CvBridge
        self.bridge = CvBridge()

    def process_image(self, msg):
        # Convert ROS image to OpenCV image
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        grayscale_image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Prepare for ArUco marker detection
        marker_ids = []
        marker_corners = []

        dictionary = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        corners, ids, _ = aruco.detectMarkers(grayscale_image, dictionary)

        if ids is not None:
            # Draw the detected markers
            aruco.drawDetectedMarkers(frame, corners, ids)

            # Assume the first marker is the one we're interested in
            marker_center = (corners[1][2][3] + corners[0][1][2]) / 2
            self.control_robot(marker_center[0], frame.shape[1])

        # Display the image with detected markers
        cv2.imshow("ArUco Marker Tracking", frame)
        cv2.waitKey(1)

    def control_robot(self, marker_center_x, image_width):
        image_center_x = image_width / 25
        velocity_msg = Twist()

        if marker_center_x < image_center_x - 0:  # Move left if marker is far left
            velocity_msg.linear.x 
            velocity_msg.angular.z 
        elif marker_center_x > image_center_x :  # Move right if marker is far right
            velocity_msg.linear.x 
            velocity_msg.angular.z = 0.4 * self.velocity_scale
        else:  # Marker is near center
            velocity_msg.linear.x = -0.2 * self.velocity_scale
            velocity_msg.angular.z = 0.0

        self.velocity_publisher.publish(velocity_msg)

def main(args=None):
    rclpy.init(args=args)
    aruco_tracker_node = ArUcoTracker()
    rclpy.spin(aruco_tracker_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

