import rclpy  # ROS 2 client library for Python
from rclpy.node import Node  # Base class for your own nodes
from sensor_msgs.msg import Image  # Message type for image data
from cv_bridge import CvBridge  # Tool to convert ROS <-> OpenCV images
import cv2  # OpenCV for image saving
import os  # To handle file paths

from your_module_containing_aaaaa import AAAAA  # 👈 Change this to the actual filename/module where your AAAAA class is

class ImageSaverNode(Node):
    def __init__(self):
        super().__init__('image_saver_node')  # Initialize the ROS node with a name

        # Subscribe to the image topic from Gazebo/ARIAC camera
        # 👇 Change the topic to match your camera topic in ARIAC if needed
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',  # Replace with your actual topic
            self.listener_callback,  # Callback function when new image is received
            10  # Queue size
        )

        self.bridge = CvBridge()  # Bridge to convert ROS Image -> OpenCV image

        # AAAAA class parses message strings and generates filenames
        self.part_parser = AAAAA()

        self.image_index = 0  # Counter to number saved images

        # Set output directory for dataset
        self.output_dir = os.path.join(os.getcwd(), 'dataset')
        os.makedirs(self.output_dir, exist_ok=True)  # Make the dataset folder if it doesn't exist

    def listener_callback(self, msg):
        # Convert ROS Image message to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Get part info (normally you'd match this with the current state in Gazebo)
        # 👇 Here we fake it by cycling through known messages in AAAAA
        part_msg = self.part_parser.get_one_msg(self.image_index % len(self.part_parser.get_msg()))

        # Clean the message (turn string into dict with color, type, etc.)
        cleaned = self.part_parser.cleanUpPartMsg(part_msg)

        # Generate a descriptive filename like 'pump_red_000.png'
        filename = self.part_parser.generate_image_filename(cleaned, index=self.image_index)

        # Full path to where the image will be saved
        path = os.path.join(self.output_dir, filename)

        # Save image as PNG
        cv2.imwrite(path, cv_image)

        # Print log message to terminal
        self.get_logger().info(f'Saved image: {path}')

        # Increase index for next image
        self.image_index += 1


def main(args=None):
    # Initialize ROS 2 system
    rclpy.init(args=args)

    # Create the node
    node = ImageSaverNode()

    # Keep it alive and processing callbacks
    rclpy.spin(node)

    # Shutdown cleanly
    node.destroy_node()
    rclpy.shutdown()
