# Robotics-AI-in-Agriculture-Docker-ROS2-
Automated Plant Disease Classification and Color Extraction using Machine Learning Models in ROS2 Docker Environment.

![Overall Project Architecture](https://github.com/Dherya27/Robotics-AI-in-Agriculture-Docker-ROS2-/blob/main/Overall_Project_Architecture.001.jpeg)

### Google Colab link where I trained my model: 

 1. https://colab.research.google.com/drive/1eVDuzPs5H3M4FztH42GFGmBATmVBlkEZ?usp=share_link

### I have used the dataset from Kaggle : 

 1. https://www.kaggle.com/datasets/rashikrahmanpritom/plant-disease-recognition-dataset?resource=download

### Software, Tools, Librarires requirements:

 1. Docker
 2. ROS2 (Humble)
 3. Visual Studio Code
 4. Google Colab
 5. Python3
 6. Numpy
 7. Matplotlib
 8. PyTorch
 9. Scikit-Learn
 10. KMeans Clustering
 11. EfficientNet B0 model
 12. OpenCV

### Procedure :

   Clone my repository, naviagte the directory in your terminal where my Dockerfile is present.

   Run the command following commands: 
   1. docker build -t <your_image_name> .
   2. docker run -it --net=detect <your_image_name> bash 
   
   Now, you will enter into the docker container. Note the docker conatiner ID, Run the following command in the two other new terminals to enter that conatiner:
   1. docker exec -it <conatiner ID> bash  

   Now, run the following commands in the both terminal:
   1. source /opt/ros/humble/setup.bash
   2. navigate to the ros2_ws and run the command following commands:
       a) pip install efficientnet_pytorch
       b) pip install scikit-Learn
       c) colcon build --symlink-install  
   3. go to the home( by command-- cd .. ), now run the command -- source /root/ros2_ws/install/setup.bash

   Now, Run the command in:
   1. first terminal : ros2 run plant_disease publisher 
   2. second terminal : ros2 run plant_disease subscriber_1
   3. third terminal : ros2 run plant_disease subscriber_2 

   you will get the publishing message, predicting message and color_extracting message.
   ![Publisher terminal](https://github.com/Dherya27/Robotics-AI-in-Agriculture-Docker-ROS2-/blob/main/Publisher.png)
   ![Subscriber_1 terminal](https://github.com/Dherya27/Robotics-AI-in-Agriculture-Docker-ROS2-/blob/main/Subscriber_1.png)
   ![Subscriber_2 terminal](https://github.com/Dherya27/Robotics-AI-in-Agriculture-Docker-ROS2-/blob/main/Subscriber_2.png)

### Feel free to contact me if you find any problem:
     dheryarobotics27@gmail.com





