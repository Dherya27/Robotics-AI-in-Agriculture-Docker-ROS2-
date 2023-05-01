#!/usr/bin/python3

# Importing required libraries;
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import matplotlib.pyplot as plt
from collections import Counter
from sklearn.cluster import KMeans

# This function takes RGB values as input and returns the HEX value of the color;
def RGB2HEX(color):
    return "#{:02x}{:02x}{:02x}".format(int(color[0]), int(color[1]), int(color[2]))

# This function takes a ROS Image message and returns the OpenCV image;
def get_image(image_msg):
    bridge = CvBridge()
    image = bridge.imgmsg_to_cv2(image_msg, desired_encoding="passthrough")
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    return image

# This function takes an OpenCV image, the number of colors to extract, and optional parameters for visualization,
 # performs KMeans clustering and returns the ordered RGB, HEX and their counts for the colors in the image.
def get_colors(image, number_of_colors, show_chart, chart_name):
    modified_image = cv2.resize(image, (600, 400), interpolation=cv2.INTER_AREA)
    modified_image = modified_image.reshape(modified_image.shape[0] * modified_image.shape[1], 3)
    clf = KMeans(n_clusters=number_of_colors)  # performing KMeans clustering on the image;
    labels = clf.fit_predict(modified_image)
    counts = Counter(labels)
    counts = dict(sorted(counts.items()))
    center_colors = clf.cluster_centers_

    # Colors are ordered by iterating through the keys for piechart generation;
    ordered_colors = [center_colors[i] for i in counts.keys()]
    hex_colors = [RGB2HEX(ordered_colors[i]) for i in counts.keys()]
    rgb_colors = [ordered_colors[i] for i in counts.keys()]
    
    # Plotting piechart for color distribution;
    if show_chart:
        plt.figure(figsize=(5, 3))
        plt.pie(counts.values(), labels=hex_colors, colors=hex_colors)
        plt.savefig(os.path.join('/root/ros2_ws/src/extracted_color_piechart', f"{chart_name}.png"))

    return rgb_colors

 # This class defines the subscriber node that listens to the 'images' topic and extracts colors from the incoming images;
class Subscriber(Node):
    def __init__(self):
        super().__init__('subscriber')
        self.subscription = self.create_subscription(Image, 'images', self.listener_callback, 10)
        self.subscription  
        self.counter = 0
     
     # This function is the callback for the subscriber, it gets called each time an image message is received and 
     # extracts colors from the image using the get_colors function;
    def listener_callback(self, msg):
        image = get_image(msg)
        colors = get_colors(image, 8, True, f"piechart{self.counter}")
        self.get_logger().info('Extracted colors from the image published by the publisher: {}'.format(colors))
        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    subscriber = Subscriber()
    rclpy.spin(subscriber)
    subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
