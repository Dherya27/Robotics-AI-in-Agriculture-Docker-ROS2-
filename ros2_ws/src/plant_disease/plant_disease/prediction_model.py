#!/usr/bin/python3

# Importing required libraries;
import os
import sys
import time
import uuid
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from PIL import Image as PIL_Image
import torch
import torchvision.transforms as transforms
#from torchvision.models import EfficientNet
from efficientnet_pytorch import EfficientNet

# Setting the device to use GPU if available;
device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

# Loading the pre-trained EfficientNet model architecture for 3 classes;
model = EfficientNet.from_pretrained('efficientnet-b0', num_classes=3).to(device)

# Loading the model weights that I have trained and saved with the name model.pth;
model.load_state_dict(torch.load('/root/ros2_ws/src/model.pth', map_location=torch.device('cpu')))

# Setting the model to evaluation mode;
model.eval()

# Defining the image transforms that should be same as we defined while training the model;
transform = transforms.Compose([
    transforms.Resize(256),
    transforms.CenterCrop(224),
    transforms.ToTensor(),
    transforms.Normalize(mean=[0.485, 0.456, 0.406],
                         std=[0.229, 0.224, 0.225])
])

# Defining the class_names in list;
class_names = ['Healthy', 'Powdery', 'Rust']

# Defining the ImageSubscriber class;
class ImageSubscriber(Node):

    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(Image, 'images', self.listener_callback, 10)
        self.subscription  
     
     # A callback function that is called whenever a new image message is received;
    def listener_callback(self, msg):
        
        data = bytes(msg.data)  # convert the raw image message data to bytes;
        pil_image = PIL_Image.frombytes("RGB", (msg.width, msg.height), data)  # Create a PIL image object from the byte data and the image dimensions;
        preprocessed_image = transform(pil_image).to(device)  # Preprocess the image using a predefined transformation and move it to a device for prediction;

         # Making a prediction on the preprocessed image using a pretrained model;
        with torch.no_grad():
            logits = model(preprocessed_image.unsqueeze(0)) # Get the logits from the model for the image;
            probabilities = torch.softmax(logits, dim=1) # Compute the probabilities of the classes using a softmax function
            confidence, predicted_class = torch.max(probabilities, dim=1) # Get the index of the class with the highest probability as the predicted class;
            confidence = float(confidence.item()) # Convert the predicted class and confidence to Python scalar values;
            predicted_class = predicted_class.item()
            #print("Predicted Class: ", predicted_class)
            #print("Confidence: ", confidence)


         # Saving the predicted image to a directory with a filename containing the predicted class and timestamp;
        timestamp = time.strftime('%Y%m%d_%H%M%S')
        filename = f'{class_names[predicted_class]}_{timestamp}.jpg'
        filepath = os.path.join('/root/ros2_ws/src/predicted_images', filename)
        pil_image.save(filepath)

         # Publishing the predicted class and confidence as a message to a ROS2 topic named 'prediction';
        prediction_msg = String()
        prediction_msg.data = f"Predicted class is {class_names[predicted_class]} with accuracy {confidence:.2f}."
        self.get_logger().info(prediction_msg.data)
        self.pub.publish(prediction_msg)
        
 # A main function that initializes the ROS2 environment, creates an ImageSubscriber object, and spins the node;
def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriber()
    node.pub = node.create_publisher(String, 'prediction', 10)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
