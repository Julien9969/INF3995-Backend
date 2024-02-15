FROM ubuntu:22.04
ARG DEBIAN_FRONTEND=noninteractive

RUN apt update && apt upgrade -y
RUN apt install net-tools
# INSTALL UTILS
RUN apt update && apt install lsb-release wget curl gnupg python3-pip git -y

# OPENGL/MESA UTILS
RUN apt update && apt install mesa-utils libglu1-mesa-dev freeglut3-dev mesa-common-dev -y


# # INSTALL IGNITION GAZEBO
# RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
# RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# RUN apt update && apt install ignition-fortress -y


# INSTALL ROS2
RUN apt update && apt install software-properties-common -y
RUN pip3 install vcstool colcon-common-extensions
RUN add-apt-repository universe

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN apt update; apt install ros-humble-desktop -y

RUN apt-get clean 
RUN apt update && apt install python3-rosdep2 -y
RUN sh -c 'echo "deb [arch=amd64,arm64] http://repo.ros2.org/ubuntu/main `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN apt install python3-colcon-common-extensions -y

RUN apt update && apt install -y vim
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
RUN echo "export GZ_VERSION=fortress" >> /root/.bashrc

RUN mkdir /src/app/ros_nodes -p
WORKDIR /src/app

COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

COPY deploy-backend.sh /src/app/ros_nodes/deploy-backend.sh
RUN chmod +x /src/app/ros_nodes/deploy-backend.sh

COPY start-app.sh /
RUN chmod +x /start-app.sh
