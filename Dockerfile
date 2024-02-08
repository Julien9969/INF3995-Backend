# FROM tiangolo/uvicorn-gunicorn-fastapi:python3.8
FROM ubuntu:22.04
ARG DEBIAN_FRONTEND=noninteractive

RUN apt update; apt upgrade -y
# INSTALL UTILS
RUN apt install lsb-release wget curl gnupg python3-pip git -y

# INSTALL ROS2
RUN apt install software-properties-common -y
RUN pip3 install vcstool colcon-common-extensions
RUN add-apt-repository universe

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN apt update; apt install ros-humble-desktop -y

RUN pip3 install rosdep

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc


WORKDIR /src
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt
# COPY ./app ./app
# CMD ["uvicorn", "app.main:app", "--host", "0.0.0.0", "--port", "80"]
