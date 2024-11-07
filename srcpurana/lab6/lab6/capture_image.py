import rclpy
import cv2
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class Capture(Node):
    def __init__(self):
        super().__init__('video_subscriber')
        self.create_subscription(
            Image,
            '/camera1/image_raw',
            self.process_data,
            10
        )
        self.out = cv2.VideoWriter(
            '/home/aryavarta/output.avi',
            cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'),
            10,
            (512, 512)
        )
        self.bridge = CvBridge()

    def process_data(self, data):
        try:
            frame = self.bridge.imgmsg_to_cv2(data)
            # Optionally resize the frame to (512, 512) if needed
            frame = cv2.resize(frame, (512, 512))
            self.out.write(frame)
            cv2.imwrite('/home/aryavarta/shot.png', frame)  # Save the image
            cv2.imshow("output", frame)
            cv2.waitKey(1)  # Use a small delay
        except Exception as e:
            self.get_logger().error(f"Failed to process image: {str(e)}")

    def __del__(self):
        if hasattr(self, 'out'):
            self.out.release()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = Capture()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
