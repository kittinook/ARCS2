# LAB1 – Command Line Interface with Motorsim and Turtlesim

## Prerequisites

Ensure you have ROS 2 Humble installed. If rosdep is not initialized, it will be set up automatically.

## System Architecture

![Alt text](/ARCS2.jpg "System Architecture")

## Installation

Clone the repository and install dependencies:

```bash
git clone -b LAB1 https://github.com/kittinook/ARCS2.git
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
sudo apt install ros-humble-teleop-*
sudo apt install texlive-xetex texlive-fonts-recommended texlive-plain-generic

colcon build
source install/setup.bash
```

## Instructions

Create your GitHub repository following these guidelines:
- Repository name: `FRA502-LAB-StudentID`
- Description: `Name Surname StudentID (Nickname)` 
- Choose visibility: `Public`
- Add README: `On`

Example:
- `FRA502-LAB-6503`
- `Keerati Ubolmart 6503 (Beam)`

Then create a `branch` named: `LAB1` 

**When users run** `git clone -b LAB1 [YOUR_GIT_LINK]`, the file structure must be as follows:

```
FRA502-LAB-StudentID/
├── Exercise1.ipynb
├── Exercise2.ipynb
└── README.md
```

[Click here to create a repository](https://github.com/new)

## Important Notes
- Prohibit from accessing any files within the `src` directory.
- Complete `Exercise1.ipynb` between 13:45 and 14:15. Commit your work by 14:20 19 AUG 2025 GMT+7.
- Complete `Exercise2.ipynb` between 14:20 and 15:50. Commit your work by 16:30 19 AUG 2025 GMT+7.
- Ensure you submit your Git link in Google Classroom.
- During the lab, you can access the internet as usual, but all Generative AI, including VSCode's Copilot and others, are prohibited.
- Submissions are based on **the time of the last commit**.
- **WARNING: If you do not follow these instructions exactly, your score will be 0 in all cases.**

Make an appointment for an examination by [Click here to queue](https://docs.google.com/spreadsheets/d/102x7QDbCxpxB_BmuFilWCqS9xsuPyOtY7FSXAnwPJss/edit?usp=sharing)