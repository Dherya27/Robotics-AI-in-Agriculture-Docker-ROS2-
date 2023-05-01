#!/usr/bin/python3

# Importing required libraries;
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import cv2
import time
import io

# Initializing an instance of the CvBridge class;
bridge = CvBridge ()

# Defining a Publisher class that inherits from the Node class;
class Publisher(Node):
    
     # Defining a constructor for the Publisher class;
    def __init__(self):
        super().__init__('publisher')
        self.publisher_ = self.create_publisher(Image, 'images', 10)  # creating a publisher that publishes messages of type Image on the 'images' topic;
        timer_period = 5
        self.timer = self.create_timer (timer_period, self.timer_callback)  # Create a timer that calls the timer_callback method every timer_period seconds;
        self.i = 1
    
     # The method that is called by the timer;
    def timer_callback(self):

         # Defining the images directory where my image files are saved in .jpg;
        image_path = '/root/images/'
        try:
            file_name = f'{image_path}{self.i}.jpg'
            read_img = cv2.imread(file_name)  # read in the image using OpenCV's imread function;
            img = bridge.cv2_to_imgmsg(read_img, encoding="passthrough")  # convert the OpenCV image to a ROS2 Image message using the CvBridge class;
            self.publisher_.publish (img)  # puublish the image message on the ros2 topic 'images';
            self.get_logger().info('Publishing image message: "%d"' % self.i)  # log a message to indicate that the image has been published;
            self.i += 1
        except Exception as e:
            print (e)

def main(args=None):
    rclpy.init(args=args) # initialize the ROS2 python client library;
    publisher = Publisher()
    rclpy.spin(publisher)  # start the node and spin it to wait for incoming messages;
    publisher.destroy_node()
    rclpy.shutdown()  # shutdown the ROS2 python client library;

if __name__ == '__main__':
    main()
