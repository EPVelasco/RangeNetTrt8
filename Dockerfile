FROM nvidia/cuda:11.4.0-cudnn8-devel-ubuntu20.04

# update apt
RUN sed -i s@/archive.ubuntu.com/@/mirrors.tuna.tsinghua.edu.cn/@g /etc/apt/sources.list \
    && apt update \
    && DEBIAN_FRONTEND=noninteractive apt install -y \
        apt-utils \
        bash-completion \
        build-essential \
        ca-certificates \
        cmake \
        curl \
        git \
        keyboard-configuration \
        libx11-dev \
        locales \
        lsb-core \
        mlocate \
        nano \
        net-tools \        
        openssh-server \
        software-properties-common \
        vim \
        wget \
        gcc-9 g++-9 gcc-10 g++-10 \
    && rm -rf /var/lib/apt/lists/*

# ROS noetic
RUN DEBIAN_FRONTEND=noninteractive  sh -c '. /etc/lsb-release && echo "deb http://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu/ `lsb_release -cs` main" > /etc/apt/sources.list.d/ros-latest.list' \
    && apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 \
    && apt update \
    && DEBIAN_FRONTEND=noninteractive apt install -y ros-noetic-desktop-full \
    && echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc \
    && DEBIAN_FRONTEND=noninteractive apt install -y \
        python3-rosdep \
        python3-rosinstall \
        python3-rosinstall-generator \
        python3-wstool \
        python3-pip \
        && rm -rf /var/lib/apt/lists/*

#Update ROS
RUN apt update \
    && DEBIAN_FRONTEND=noninteractive apt install -y \
        libboost-all-dev \
        libboost-python-dev \
        libeigen3-dev \
        libgeographic-dev \
        libglfw3-dev \
        libglm-dev \
        libgtest-dev \
        libpugixml-dev \
        python3-catkin-tools \
        libpcap-dev \
        ros-noetic-angles \
        ros-noetic-camera-info-manager \
        ros-noetic-ddynamic-reconfigure \
        ros-noetic-diagnostic-updater \
        ros-noetic-geodesy \
        ros-noetic-jsk-recognition-msgs ros-noetic-visualization-msgs \
        ros-noetic-lanelet2 \
        ros-noetic-nav-msgs \
        ros-noetic-nmea-msgs \
        ros-noetic-tf2-sensor-msgs \
    # catkin build
    && rm -rf /var/lib/apt/lists/* \
    && pip3 install osrf-pycommon

# Install other apt package dependencies
RUN apt update \
    && DEBIAN_FRONTEND=noninteractive apt install -y \
        htop \
        wondershaper \
        ethstatus \
        cowsay \
    && rm -rf /var/lib/apt/lists/*

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

#  install gtsam,osqp,ceres >>>
RUN  cd /tmp \
    && git clone --depth=1 https://github.com/borglab/gtsam.git gtsam \
    && cd gtsam \
    && mkdir build && cd build && cmake .. \
    && make -j$(nproc) && make install \
    && DEBIAN_FRONTEND=noninteractive apt update \
    && apt install -y libgoogle-glog-dev libgflags-dev libatlas-base-dev libeigen3-dev libsuitesparse-dev \
    && wget -c http://ceres-solver.org/ceres-solver-2.0.0.tar.gz -O /tmp/ceres-solver-2.0.0.tar.gz \
    && cd /tmp \
    && tar -xzvf ceres-solver-2.0.0.tar.gz \
    && cd /tmp/ceres-solver-2.0.0 \
    && mkdir build && cd build && cmake .. \
    && make -j$(nproc) \
    && make install \
    && cd /tmp/ \
    && git clone --depth=1 --recursive https://github.com/osqp/osqp osqp \
    && cd osqp && mkdir build && cd build \
    && cmake .. \
    && make -j$(nproc) \
    && make install \
    && rm -rf /tmp/*


RUN pip3 install --no-cache-dir \
    catkin_pkg \
    netifaces \
    rospkg \
    defusedxml \
    easydict \
    empy \
    fire \
    gdown \
    matplotlib \
    opencv-python \
    pandas \
    pygeodesy \
    tqdm \
    pybind11[global] \
    && apt update \
    && DEBIAN_FRONTEND=noninteractive apt install -y \
        python3-pyqt5.qtserialport

# >>> Pytorch >>>
RUN pip3 install --no-cache-dir torch==1.10.1+cu113 torchvision==0.11.2+cu113 torchaudio==0.10.1+cu113 -f https://download.pytorch.org/whl/cu113/torch_stable.html
RUN apt update \
    && DEBIAN_FRONTEND=noninteractive apt install -y \
    libnvinfer8 \
    libnvonnxparsers8 \
    libnvparsers8 \
    libnvinfer-plugin8 \
    libnvinfer-dev \
    libnvonnxparsers-dev \
    libnvparsers-dev \
    libnvinfer-plugin-dev

# Now create the same user as the host itself
ARG UID=1000
ARG GID=1000
RUN addgroup --gid ${GID} ros
RUN adduser --gecos "ROS User" --disabled-password --uid ${UID} --gid ${GID} ros
RUN usermod -a -G dialout ros
RUN mkdir config && echo "ros ALL=(ALL) NOPASSWD: ALL" > config/99_aptget
RUN cp config/99_aptget /etc/sudoers.d/99_aptget
RUN chmod 0440 /etc/sudoers.d/99_aptget && chown root:root /etc/sudoers.d/99_aptget

# Change HOME environment variable
ENV HOME /home/epvs_docker
RUN mkdir -p ${HOME}/ros_ws/src

# Initialize the workspace
RUN cd ${HOME}/ros_ws/src && /bin/bash -c "source /opt/ros/noetic/setup.bash; catkin_init_workspace"
RUN cd ${HOME}/ros_ws /bin/bash -c "source source /opt/ros/noetic/setup.bash; catkin_make"

# clone repository
RUN cd ${HOME}/ros_ws/src/ && git clone https://github.com/EPVelasco/RangeNetTrt8.git && cd ${HOME}/ros_ws/ 

# Copy files
RUN mkdir -p ${HOME}/ros_ws/libtorch
COPY ./libtorch ${HOME}/ros_ws/libtorch/
COPY ./darknet53 /home/epvs_docker/ros_ws/src/RangeNetTrt8/darknet53/

# build repository
RUN cd ${HOME}/ros_ws/ && /bin/bash -c "source /opt/ros/noetic/setup.bash; catkin init"
RUN cd ${HOME}/ros_ws/ && /bin/bash -c "source /opt/ros/noetic/setup.bash; catkin build;"

RUN cd ${HOME}/ros_ws/ && /bin/bash -c "source devel/setup.bash" 


## Run in docker terminal 
## first time ## It takes a few minutes to get the model.trt file

	#cd
	#cd ros_ws 
	#./devel/lib/rangenet_plusplus/infer

## When the .trt file is ready, run this command
	# source devel/setup.bash
	#roslaunch rangenet_plusplus rangenet.launch



