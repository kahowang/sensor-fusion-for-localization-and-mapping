FROM ubuntu:18.04

# ------ powered by Ge Yao, alexgecontrol@qq.com ------

LABEL maintainer="alexgecontrol@qq.com"

# ------ USER ROOT HAS BEEN ACTIVATED ------

# use root for dependency installation:
USER root

# ------ PART 0: set environment variables ------

# set up environment:
ENV DEBIAN_FRONTEND noninteractive
ENV LANG=C.UTF-8 LC_ALL=C.UTF-8
ENV HOME=/root SHELL=/bin/bash

# ------ PART 1: set up CN sources ------

# Ubuntu:
COPY ${PWD}/image/etc/apt/sources.list /etc/apt/sources.list
RUN rm -f /etc/apt/sources.list.d/*

# Python: 
COPY ${PWD}/image/etc/pip.conf /root/.pip/pip.conf 

# ------ PART 2: set up apt-fast -- NEED PROXY DUE TO UNSTABLE CN CONNECTION ------

# install:
RUN apt-get update -q --fix-missing && \
    apt-get install -y --no-install-recommends --allow-unauthenticated \
        # PPA utilities:
        software-properties-common \
        # certificates management:
        dirmngr gnupg2 \
        # download utilities:
        axel aria2 && \
    apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-keys 1EE2FF37CA8DA16B && \
    add-apt-repository ppa:apt-fast/stable && \
    apt-get update -q --fix-missing && \
    apt-get install -y --no-install-recommends --allow-unauthenticated apt-fast && \
    rm -rf /var/lib/apt/lists/*

# CN config:
COPY ${PWD}/image/etc/apt-fast.conf /etc/apt-fast.conf

# ------ PART 3: add external repositories ------

# ROS:
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
COPY ${PWD}/image/etc/apt/sources.list.d/ /etc/apt/sources.list.d/
# libsparse:
RUN add-apt-repository -r ppa:bzindovic/suitesparse-bugfix-1319687

# ------ PART 4: install packages ------

RUN apt-fast update --fix-missing && \
    apt-fast install -y --no-install-recommends --allow-unauthenticated \
        # package utils:
        sudo dpkg pkg-config apt-utils \
        # security:
        openssh-server pwgen ca-certificates \
        # network utils:
        curl wget iputils-ping net-tools \
        # command line:
        vim grep sed patch \
        # io:
        pv zip unzip bzip2 \
        # version control:
        git mercurial subversion \
        # daemon & services:
        supervisor nginx \
        # dev. tools:
        terminator \
        firefox \
        # potential image & rich text IO:
        libsdl1.2-dev libsdl-net1.2-dev libsdl-image1.2-dev \
        lxde \
        gnome-themes-standard \
        xvfb dbus-x11 x11-utils libxext6 libsm6 x11vnc \
        gtk2-engines-pixbuf gtk2-engines-murrine pinta ttf-ubuntu-font-family \
        mesa-utils libgl1-mesa-dri libxrender1 \
        gnuplot \
        texlive-latex-extra \
        #
        # general development:
        #
        # a. c++:
        gcc g++ \
        make cmake build-essential autoconf automake libtool \
        libglib2.0-dev libboost-dev libboost-all-dev \
        libomp-dev libtbb-dev \
        libgoogle-glog-dev \
        # b. Python 2:
        python-pip python-dev python-tk \
        # c. lua:
        lua5.3 liblua5.3-dev libluabind-dev \
        # 
        # numerical optimization:
        # 
        coinor-libcoinutils-dev \
        coinor-libcbc-dev \
        libeigen3-dev \
        gfortran \
        libopenblas-dev liblapack-dev \
        libdw-dev libatlas-base-dev libsuitesparse-dev \
        libcholmod3 libcxsparse3 \
        libmetis-dev \
        #
        # 3D graphics:
        #
        freeglut3-dev \
        libqt4-dev libqt4-opengl-dev \
        qt5-default qt5-qmake \
        qtdeclarative5-dev libqglviewer-dev-qt5 \
        #
        # ROS melodic:
        # 
        ros-melodic-desktop-full \
        ros-melodic-ecl-threads \
        ros-melodic-rosbridge-server \
        ros-melodic-tf2 ros-melodic-tf2-ros ros-melodic-tf2-sensor-msgs \
        ros-melodic-teleop-twist-keyboard \
        ros-melodic-rviz-visual-tools \
        ros-melodic-plotjuggler \ 
        python-catkin-tools python-rosdep python-rosinstall python-rosinstall-generator python-wstool \
        ninja-build \
        # imu:
        ros-melodic-imu-complementary-filter ros-melodic-imu-filter-madgwick ros-melodic-rviz-imu-plugin \
        # lidar:
        ros-melodic-laser-pipeline \
        ros-melodic-perception-pcl && \
    apt-fast autoclean && \
    apt-fast autoremove && \
    rm -rf /var/lib/apt/lists/*

# ordered startup fix for supervisord:
RUN pip install ordered-startup-supervisord

# ------ PART 5: offline installers ------

# load installers:
COPY ${PWD}/installers /tmp/installers
WORKDIR /tmp/installers

# install Google Protobuf latest:
RUN git clone https://github.com/google/protobuf.git -o protobuf && cd protobuf && \
    # sync:
    git submodule update --init --recursive && \
    # config:
    ./autogen.sh && ./configure && \ 
    # build:
    make -j8 && \
    # install:
    make install

# install Aceinna GNSS/IMU sim IMU-GNSS-Odo simulation with customized error modes:
RUN git clone https://github.com/AlexGeControl/GNSS-INS-SIM-Extended.git && cd GNSS-INS-SIM-Extended && \
    # install:
    python setup.py install

# install GeographicLib -- https://geographiclib.sourceforge.io/html/install.html
RUN wget https://nchc.dl.sourceforge.net/project/geographiclib/distrib/GeographicLib-1.50.1.zip && \
    unzip -q GeographicLib-1.50.1.zip && cd GeographicLib-1.50.1 && \
    mkdir build && cd build && \
    # config:
    cmake .. && \
    # build:
    make -j8 && \
    # install:
    make install

# install sophus -- https://github.com/strasdat/Sophus:
RUN git clone https://github.com/strasdat/Sophus.git -o Sophus && cd Sophus && \
    mkdir build && cd build && \
    # config:
    cmake .. && \
    # build:
    make -j8 && \
    # install:
    make install

# install ceres:
RUN git clone https://github.com/ceres-solver/ceres-solver.git -o ceres-solver && \
    mkdir ceres-bin && cd ceres-bin && cmake ../ceres-solver && \
    make -j8 && make install

# install g2o -- https://github.com/RainerKuemmerle/g2o:
RUN git clone https://github.com/RainerKuemmerle/g2o.git -o g2o && cd g2o && \
    mkdir build && cd build && \
    # config:
    cmake .. && \
    # build:
    make -j8 && \
    # install:
    make install

# install gtsam -- https://github.com/borglab/gtsam.git
RUN git clone https://github.com/borglab/gtsam.git -o gtsam && cd gtsam && \
    mkdir build && cd build && \
    # config:
    cmake .. && \
    # build:
    make -j8 && \
    # install:
    make install

# install tini:
RUN chmod u+x ./download-tini.sh && ./download-tini.sh && dpkg -i tini.deb && \
    apt-get clean

RUN rm -rf /tmp/installers


# ------ PART 6: set up ROS environments ------

# initialize rosdep
# 
# NOTE:
# be careful about DNS resolution problem caused by https://raw.githubusercontent.com
# get the latest IP address of the site from Baidu and Google search engine
# 
RUN rosdep fix-permissions && \
    rosdep init && \
    rosdep update

# activate ros environment:
RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc

# for remote debugging:
EXPOSE 11311

# ------ PART 6: set up VNC servers ------

COPY image /

WORKDIR /usr/lib/

RUN git clone https://github.com/novnc/noVNC.git -o noVNC

WORKDIR /usr/lib/noVNC/utils

RUN git clone https://github.com/novnc/websockify.git -o websockify

WORKDIR /usr/lib/webportal

RUN pip install --upgrade pip && pip install -r requirements.txt

EXPOSE 80 5901 9001

# ------ PART 7: set up Sensor Fusion courseware dependencies ------

COPY environment /workspace

WORKDIR /workspace 

RUN pip install -r requirements.txt

# ------------------ DONE -----------------------

# enable dependency lib linking:
ENV LD_LIBRARY_PATH=/usr/local/lib

ENTRYPOINT ["/startup.sh"]