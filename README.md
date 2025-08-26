# LAB2 - Eater vs. Killer: Spawn–Forage–Pursuit (SFP) with RViz2 and Turtlesim+

An interactive lab implementing user-spawned “pizza” targets, service-driven turtle lifecycle management, and RViz2 click-to-pose evasion/pursuit behaviors using `turtlesim`.

## Important Notes
- Complete all of this and **commit your work by 16:00 26 AUG 2025 GMT+7**.
- Ensure you submit your Git link in Google Classroom.
- During the lab, you can access the internet as usual, but all Generative AI, including VSCode's Copilot and others, are prohibited.
- Submissions are based on **the time of the last commit**.
- **WARNING: If you do not follow these instructions exactly, your score will be 0 in all cases.**

Make an appointment for an examination by [Click here to queue](https://docs.google.com/spreadsheets/d/102x7QDbCxpxB_BmuFilWCqS9xsuPyOtY7FSXAnwPJss/edit?usp=sharing)

---

# Demo and Instructions Video

[![Watch the video](https://i9.ytimg.com/vi_webp/keqN5zx5Sp0/mq3.webp?sqp=CNznssUG-oaymwEmCMACELQB8quKqQMa8AEB-AH-CYAC0AWKAgwIABABGH8gFCgTMA8=&rs=AOn4CLDeMJL9i7qx6XohPcvN_zs_BBbCaQ)](https://youtu.be/keqN5zx5Sp0)


## Part 1 - Complete System Architecture

[Download the System Architecture](./LAB2_SA.pdf)

- Draw a complete system architecture diagram.  
- The diagram must clearly show the connections between all nodes, including topics and services.  
- For each node, indicate which components are **Publishers**, **Subscribers**, and **Service Clients**.  

---

## Part 2 - Build All Nodes

**When users run** `git clone -b LAB2 [YOUR_GIT_LINK]`, the file structure must be as follows:



```
FRA502-LAB-StudentID/
├── src/
│ ├── lab2.rviz
│ ├── lab2
│ │ ├── CMakeLists.txt
│ │ ├── include/
│ │ ├── lab2/
│ │ ├── package.xml
│ │ ├── scripts
│ │ │ ├── eater.py
│ │ │ ├── killer.py
│ │ │ └── turtlesim_pose.py
│ │ └── src/
│ └── turtlesim_plus/
│── LAB2_SA.pdf
└── README.md

```

- Implement **three main nodes** to satisfy the architecture:  
  - `eater` (handles pizza detection and movement to spawned pizza)  
  - `killer` (pursues eater after all pizzas are consumed)  
  - `turtlesim_pose` (monitors pose and provides position feedback)  


---
### Node `eater` requirement
### What this node **must do**
- **Control turtle1 to target position** by publishing velocity to `/turtle1/cmd_vel` using pose feedback from `/turtle1/pose` for navigation.
- **Accept click targets** from `/mouse_position` and `/goal_pose` and convert to turtlesim coordinates.
- **Forage mode operation** by continuously spawning pizzas at clicked locations via spawn service and eating them in order-by-order sequence using `/turtle1/eat` service - must handle simultaneous pizza spawning while executing eating sequence.
- **Track progress** via `/turtle1/pizza_count` to determine when all pizzas are consumed.
- **Evade mode operation** (when all pizzas eaten) robot must move to the target from `/mouse_position` and `/goal_pose` ..
- **Node must be able to run by `ros2 run` in Terminal**.

### Node `killer` requirement  
### What this node **must do**
- **Control turtle2 to target position** by publishing velocity to `/turtle2/cmd_vel` using pose feedback from `/turtle2/pose`.
- **Track eater target** by subscribing to `/turtle1/pose` as moving pursuit target after all pizzas are eaten.
- **Terminate on capture** by calling `/remove_turtle` service when close enough to eater, then stop motion.
- **Node must be able to run by `ros2 run` in Terminal**.

### Node `turtlesim_pose` requirement
### What this node **must do**
- **Subscribe to turtle poses** from both **`/turtle1/pose`** and **`/turtle2/pose`** (`turtlesim/Pose`) to track the positions of both turtles in real-time.
- **Map default grid interface** with 10x10m dimensions that properly interfaces with the turtlesim+ GUI coordinate system for visualization and interaction.
- **Publish odometry data** by creating publishers for **`/odom1`** and **`/odom2`** (`nav_msgs/Odometry`) to provide proper odometry information for both turtle1 and turtle2.
- **Broadcast transforms** between the `odom` frame and individual turtle frames (`turtle1` and `turtle2`) using the TF2 system to maintain proper coordinate frame relationships.
- **Implement odometry publishing function** that handles publishing odometry data to `/odom1`, `/odom2`, and corresponding `/tf` transforms for both turtles, following the example structure: `def example_pub(self, msg, turtle_name, child_frame_id)`.
- **RViz2 compatibility** by ensuring the node works seamlessly with the provided `config fun2.rviz` configuration file for proper visualization.
- **Node must be able to run by `ros2 run` in Terminal** with standard ROS2 command-line interface.

## **If the TA finds any issues, such as a node running but not meeting the requirements etc. , you will receive half the points for that issue in each node requirement.**

---

## Part 3 - Your Turn

Download the README.md file then fill your command for run all node when TA tests to you

Below is example

1. **Clone the repository** (replace `StudentID` with your own ID):
   ```bash
   git clone -b LAB2 https://github.com/<your-org>/FRA502-LAB-StudentID.git
    ```
2. **Run all nodes** (replace `StudentID` with your own ID):
   ```bash
   ros2 run FRA502-LAB-StudentID <node_name>
   ```



Modify the commands to match your package and node names. below here