#!/bin/bash

set -e

export DEBIAN_FRONTEND=noninteractive

echo "========================================"
echo ".... Installing ROS 2 Humble ...."
echo "========================================"

sleep 2

# ---------------------------------------------------------
# Check Ubuntu version
# ---------------------------------------------------------

os_codename=$(lsb_release -cs)

if [ "$os_codename" = "jammy" ]; then
    echo "--------------------------------"
    echo ".... Ubuntu 22.04 Jammy detected ...."
    echo ".... Can install ROS 2 Humble ...."
    echo "--------------------------------"
else
    echo "--------------------------------"
    echo ".... Cannot install ROS 2 Humble ...."
    echo ".... This script requires Ubuntu 22.04 ...."
    echo "--------------------------------"
    exit 1
fi

sleep 1

# ---------------------------------------------------------
# Basic packages
# ---------------------------------------------------------

echo "========================================"
echo ".... Installing basic packages ...."
echo "========================================"

sudo apt-get update

sudo apt-get install -y \
    git \
    wget \
    vim \
    build-essential \
    lsb-core \
    lsb-release \
    net-tools \
    iputils-ping \
    curl \
    locales \
    software-properties-common \
    apt-utils

# ---------------------------------------------------------
# Locale configuration
# ---------------------------------------------------------

echo "========================================"
echo ".... Configuring locales ...."
echo "========================================"

sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

export LANG=en_US.UTF-8

locale

echo "--------------------------------"
echo ".... Locale configuration successful ...."
echo "--------------------------------"

# ---------------------------------------------------------
# Enable Universe
# ---------------------------------------------------------

echo "========================================"
echo ".... Enabling Universe repository ...."
echo "========================================"

sudo add-apt-repository --yes universe

sudo apt-get update

# ---------------------------------------------------------
# ROS 2 repository
# ---------------------------------------------------------

echo "========================================"
echo ".... Adding ROS 2 repository ...."
echo "========================================"

sudo curl -sSL \
    https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt-get update

# ---------------------------------------------------------
# ROS 2 Humble Desktop
# ---------------------------------------------------------

echo "========================================"
echo ".... Installing ROS 2 Humble Desktop ...."
echo "========================================"

sudo apt-get install -y ros-humble-desktop

echo "--------------------------------"
echo ".... ROS 2 Humble successfully installed ...."
echo "--------------------------------"

sleep 1

# ---------------------------------------------------------
# ROS development tools
# ---------------------------------------------------------

echo "========================================"
echo ".... Installing ROS development tools ...."
echo "========================================"

sudo apt-get install -y ros-dev-tools

# ---------------------------------------------------------
# Configure ROS environment
# ---------------------------------------------------------

echo "========================================"
echo ".... Configuring ROS environment ...."
echo "========================================"

ROS_BASHRC_LINE="source /opt/ros/humble/setup.bash"
TB3_BASHRC_LINE="export TURTLEBOT3_MODEL=burger"

if ! grep -qF "$ROS_BASHRC_LINE" "$HOME/.bashrc"; then
    echo "$ROS_BASHRC_LINE" >> "$HOME/.bashrc"
fi

if ! grep -qF "$TB3_BASHRC_LINE" "$HOME/.bashrc"; then
    echo "$TB3_BASHRC_LINE" >> "$HOME/.bashrc"
fi

echo "--------------------------------"
echo ".... ROS environment successfully configured ...."
echo "--------------------------------"

sleep 1

# ---------------------------------------------------------
# TurtleBot4 repository
# ---------------------------------------------------------

echo "========================================"
echo ".... Setting up RoboHub ...."
echo "========================================"

mkdir -p "$HOME/robohub"

if [ -d "$HOME/robohub/turtlebot4/.git" ]; then
    echo ".... TurtleBot4 repository already exists ...."
else
    cd "$HOME/robohub"

    echo ".... Cloning TurtleBot4 repository ...."

    git clone ist-git@git.uwaterloo.ca:robohub/turtlebot4.git
fi

# ---------------------------------------------------------
# Fast DDS
# ---------------------------------------------------------

echo "========================================"
echo ".... Installing Fast DDS configuration ...."
echo "========================================"

sudo apt-get install -y ros-humble-rmw-fastrtps-cpp

if [ -f "$HOME/robohub/turtlebot4/configs/.fastdds.xml" ]; then

    cp \
        "$HOME/robohub/turtlebot4/configs/.fastdds.xml" \
        "$HOME/.fastdds.xml"

    echo ".... Fast DDS configuration copied ...."

else

    echo "WARNING: .fastdds.xml was not found."

fi

# ---------------------------------------------------------
# TurtleBot4
# ---------------------------------------------------------

echo "========================================"
echo ".... Installing TurtleBot 4 ...."
echo "========================================"

sudo apt-get install -y ros-humble-turtlebot4-desktop

# ---------------------------------------------------------
# TurtleBot3
# ---------------------------------------------------------

echo "========================================"
echo ".... Installing TurtleBot 3 packages ...."
echo "========================================"

sudo apt-get install -y 'ros-humble-turtlebot3*'

# ---------------------------------------------------------
# Finished
# ---------------------------------------------------------

echo ""
echo "========================================"
echo ".... TurtleBot environment is correctly set up! ...."
echo "========================================"

echo ""
echo "ROS 2 Distribution:"
echo "${ROS_DISTRO:-humble}"

echo ""
echo "TurtleBot3 Model:"
echo "burger"

echo ""
echo "Installation complete!"
echo "========================================"

