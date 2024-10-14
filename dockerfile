FROM ubuntu:22.04

# ENV TZ="Asia/Kolkata"
# RUN date

# Timezone set by ROS install
# Set timezone:
# RUN ln -snf /usr/share/zoneinfo/$CONTAINER_TIMEZONE /etc/localtime && echo $CONTAINER_TIMEZONE > /etc/timezone

# setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get update && \
    apt-get install -q -y --no-install-recommends tzdata && \
    rm -rf /var/lib/apt/lists/*

# Update and install the necessary packages
# RUN apt update -y && apt upgrade -y && apt install -y python3 python3-pip python3-venv git python3-full
RUN apt update -y && apt upgrade -y && apt install -y python3 python3-pip git python3-full


    # install packages
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    dirmngr \
    gnupg2 \
    && rm -rf /var/lib/apt/lists/*

# setup sources.list
RUN echo "deb http://packages.ros.org/ros2/ubuntu jammy main" > /etc/apt/sources.list.d/ros2-latest.list

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

ENV ROS_DISTRO humble


# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    git \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

# bootstrap rosdep
RUN rosdep init && \
  rosdep update --rosdistro $ROS_DISTRO

# setup colcon mixin and metadata
RUN colcon mixin add default \
      https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml && \
    colcon mixin update && \
    colcon metadata add default \
      https://raw.githubusercontent.com/colcon/colcon-metadata-repository/master/index.yaml && \
    colcon metadata update

# install ros2 packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ros-base=0.10.0-1* \
    && rm -rf /var/lib/apt/lists/*



# Why do we need a virual environment in a docker container 
# # Create the python virtual environment
# # RUN python3 -m venv /rero/ros_bot/venv
# # RUN . /rero/ros_bot/venv/bin/activate


# Install requirements
# Despite the use of volume, this is required as volumes are set up towards the end of the container setup
# RUN mkdir /rero/ros_bot/
COPY ./requirements.txt /rero/ros_bot/requirements.txt
RUN python3 -m pip install -r /rero/ros_bot/requirements.txt

# Volume for the source
RUN mkdir -p /rero/ros_bot/app
VOLUME ["/rero/ros_bot/app"]

# ROS Bot communication port
EXPOSE 8081

COPY ./app/startup_dev.sh /rero/ros_bot/app/startup_dev.sh

# CMD [ "/bin/bash" ]
CMD ["/bin/bash", "/rero/ros_bot/app/startup_dev.sh"]