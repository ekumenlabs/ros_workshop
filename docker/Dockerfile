# Download base image ubuntu 18.04
FROM ubuntu:18.04

# Arguments for building
ARG USERID
ARG USER

# Image name
ARG "Ubuntu base for ros melodic development in ubuntu bionic."

# Setup environment
ENV TERM linux
ENV DEBIAN_FRONTEND noninteractive
RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections

# Copy installation scripts, and perform the dependency installation.
COPY docker/requirements.txt .
COPY docker/requirements_ros.txt .

RUN apt-get update && apt-get dist-upgrade --yes

RUN apt-get update && apt-get install --no-install-recommends --yes $(cat requirements.txt)

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

RUN apt-get update && apt-get install --no-install-recommends -y $(cat requirements_ros.txt)

RUN rm requirements.txt requirements_ros.txt

# Create a user with passwordless sudo
RUN adduser --uid $USERID --gecos "ekumen developer" --disabled-password $USER
RUN adduser $USER sudo
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
RUN echo "export QT_X11_NO_MITSHM=1" >> /home/$USER/.bashrc
USER $USER

# Run rosdep init and update.
RUN sudo rosdep init && rosdep update

# Creates the src folder of the workspace.
RUN mkdir -p /home/$USER/ws/src

# Adds to bashrc the ros melodic overlay sourcing.
RUN echo "source /opt/ros/melodic/setup.bash" >> /home/$USER/.bashrc

# Defines a workspace folder.
WORKDIR /home/$USER/ws

CMD ["/bin/bash"]
