# LAB1 – Command Line Interface with Motorsim and Turtlesim


## Prerequisites
Ensure you have ROS 2 Humble installed. If rosdep is not initialized, it will be set up automatically.


## Installation
Clone the repository and install dependencies:
```bash
git -b LAB1 clone https://github.com/kittinook/ARCS2.git
cd ARCS2

sudo apt update
sudo apt install -y python3-rosdep 

if ! rosdep db > /dev/null 2>&1; then
    sudo rosdep init
fi
rosdep update
rosdep install -y -i --from-paths src

sudo apt install ros-humble-rqt*
sudo apt install ros-humble-tf*
sudo apt install ros-humble-turtlesim

colcon build
source install/setup.bash
```

## Instruction

