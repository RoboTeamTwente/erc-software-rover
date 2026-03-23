import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import subprocess
from cv_bridge import CvBridge
import cv2

class VideoStreamNode(Node):
    def __init__(self):
        super().__init__('video_stream_node')
        self.bridge = CvBridge()
        
        # Subscribe to the color image topic
        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',  # ROS topic to subscribe to
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        # Convert the ROS Image message to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # Encode image to JPEG for streaming
        ret, encoded_img = cv2.imencode('.jpg', cv_image)

        if ret:
            # Run GStreamer pipeline to stream the image via UDP
            gst_command = [
                'gst-launch-1.0', 
                'appsrc', 'caps=video/x-raw,format=BGR,width=640,height=480,framerate=30/1', 
                '!', 'videoconvert', 
                '!', 'x264enc', 'tune=zerolatency', 'speed-preset=superfast', 
                '!', 'rtph264pay', 
                '!', 'udpsink', 'host=145.126.47.213', 'port=4500'
            ]
            
            # Send the image using subprocess
            gst_process = subprocess.Popen(gst_command, stdin=subprocess.PIPE)
            gst_process.communicate(input=encoded_img.tobytes())
        else:
            self.get_logger().info('Failed to encode image')

def main(args=None):
    rclpy.init(args=args)
    node = VideoStreamNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
