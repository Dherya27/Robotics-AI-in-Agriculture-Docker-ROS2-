FROM osrf/ros:humble-desktop-full

RUN apt-get update && apt-get install -y git wget python3-pip vim
RUN pip3 install setuptools==58.2.0
RUN pip3 install torch torchvision

COPY ros2_ws/ /root/ros2_ws/
COPY images/ /root/images


ENV DISPLAY=novnc:0.0

RUN apt-get update && apt-get install -y libsm6 libxext6 libxrender-dev

# Install OpenCV and its dependencies
RUN apt-get install -y python3-opencv

# Allow access to the webcam
RUN usermod -a -G video root

# Install Jupyter notebook and dependencies
RUN pip3 install jupyter jupyterlab && \
    jupyter notebook --generate-config && \
    echo "c.NotebookApp.ip = '0.0.0.0'" >> /root/.jupyter/jupyter_notebook_config.py && \
    echo "c.NotebookApp.port = 8888" >> /root/.jupyter/jupyter_notebook_config.py && \
    echo "c.NotebookApp.open_browser = False" >> /root/.jupyter/jupyter_notebook_config.py && \
    echo "c.NotebookApp.token = ''" >> /root/.jupyter/jupyter_notebook_config.py

# Install Docker CLI
RUN apt-get update && apt-get install -y apt-transport-https ca-certificates curl gnupg lsb-release
RUN curl -fsSL https://download.docker.com/linux/ubuntu/gpg | gpg --dearmor -o /usr/share/keyrings/docker-archive-keyring.gpg
RUN echo "deb [arch=amd64 signed-by=/usr/share/keyrings/docker-archive-keyring.gpg] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable" | tee /etc/apt/sources.list.d/docker.list > /dev/null
RUN apt-get update && apt-get install -y docker-ce-cli

# Create the "docker" group and allow nobody user to run Docker commands
RUN groupadd docker && usermod -aG docker nobody

# Set the working directory
WORKDIR /root

# Expose port for Jupyter notebook
EXPOSE 8888


# Start Jupyter notebook when the container starts
CMD ["jupyter", "notebook", "--allow-root", "--no-browser", "--ip=0.0.0.0"]
