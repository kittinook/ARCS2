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
│ │ │ ├── dummy_script.py
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
- **Drive the eater turtle** by publishing velocity to **`/turtle1/cmd_vel`** (`geometry_msgs/Twist`).
- **Read its own pose** from **`/turtle1/pose`** (`turtlesim/Pose`) for feedback control.
- **Accept user click targets from two sources** and convert to turtlesim coordinates:
  - GUI/canvas: **`/mouse_position`** (`geometry_msgs/Point`)
  - RViz2: **`/goal_pose`** (`geometry_msgs/PoseStamped`)
- **Forage mode (pizza remaining):**  
  - **Queue each clicked position (FIFO)** as a pizza waypoint.  
  - **Spawn a pizza** at each queued waypoint by calling **`/spawn_pizza`** (custom spawn service).  
  - **Navigate in order (no preemption in Forage)** to each queued waypoint; upon arrival:
    - **Call `/turtle1/eat`** (`std_srvs/Empty`) exactly once, then proceed to the next waypoint.
- **Track progress** via **`/turtle1/pizza_count`** (`std_msgs/Int64`) and a configurable **`max_pizza`** (parameter; may also accept runtime updates e.g., `/set_max_pizza`).

- **Evade mode (all pizzas eaten) — *synced with killer’s pursuit*:**  
  - **Stop spawning pizzas** once `pizza_count == max_pizza`.  
  - **Preempt to the latest clicked pose immediately** (single-slot “latest-only” target; no backlog).  
  - **Continuously retarget and move** toward the most recent click with **low latency (≤100 ms)** so that the killer—which follows `/turtle1/pose`—always chases a **moving goal**.  
  - **Maintain a capture tolerance consistent with the killer** (e.g., arrival if `|dx|<0.1` and `|dy|<0.1`); if captured (killer reaches tolerance and kills `turtle1`), **cease publishing** commands.  
  - **Safety while evading:** saturate linear/angular speeds, publish zero when idle or after capture, and avoid oscillations near goals.

- **Controller loop & safety (both modes):**
  - Run a timer-based control loop (≥ 100 Hz), normalize heading error to (−π, π), and clamp velocities.
  - Wait for services before use; log failures and skip unsafe commands.
- **Node must able to run by `ros2 run` in Terminal**
---

### Node `killer` requirement
### What this node **must do**
- **Drive the killer turtle** by publishing velocity to **`/turtle2/cmd_vel`** (`geometry_msgs/Twist`).
- **Read its own pose** from **`/turtle2/pose`** (`turtlesim/Pose`) for closed-loop pursuit control.
- **Track the eater turtle** by subscribing to **`/turtle1/pose`** (`turtlesim/Pose`) as the **moving target**.
- **Gate pursuit by pizza status:**
  - Subscribe to **`/turtle1/pizza_count`** (`std_msgs/Int64`) and **`/set_max_pizza`** (`std_msgs/Int64`).
  - **Remain idle** while `pizza_count < max_pizza`.
  - **Begin pursuit** immediately once `pizza_count == max_pizza`.
- **Pursuit behavior (when active):**
  - **Continuously retarget to the eater’s latest pose** (single, always-updated goal; no queue).
  - Use proportional control on distance and heading; **publish zero** when within arrival tolerance (e.g., `|dx|<0.1` and `|dy|<0.1`).
- **Terminate the eater on capture:**
  - Call **`/remove_turtle`** (`turtlesim/srv/Kill`) with name **`turtle1`** exactly once upon arrival, then stop motion.
- **Controller loop & safety:**
  - Run a timer-based control loop (≈ 100 Hz), normalize heading error to (−π, π), and avoid oscillations via bounded commands.
  - Ensure service availability before calling `/remove_turtle`; log failures and avoid repeated kill requests.
- **Node must able to run by `ros2 run` in Terminal**

---
### Node `turtlesim_pose` requirement
### What this node **must do**
- **Subscribe to turtlesim poses** for both turtles:
  - **`/turtle1/pose`** and **`/turtle2/pose`** (`turtlesim/Pose`) to receive `x, y, theta`, `linear_velocity`, and `angular_velocity`.
- **Publish odometry** for each turtle:
  - **`/odom1`** for turtle1 and **`/odom2`** for turtle2 (`nav_msgs/Odometry`), with `header.frame_id = "odom"` and timestamps from the node clock on **every pose update**.
- **Broadcast TF transforms** from the world odometry frame to each turtle:
  - Use a **single `tf2_ros::TransformBroadcaster`** to send transforms with `parent = "odom"` and `child_frame_id` equal to the turtle’s name (**`"turtle1"`**, **`"turtle2"`). 
- **Convert heading to quaternion**:
  - Compute quaternion from yaw `theta` via **`quaternion_from_euler(0, 0, theta)`** and use it in both Odometry pose and TF rotation.
- **Apply fixed map-origin offset** to positions:
  - Set odometry pose position to **`x = pose.x - 5.408`**, **`y = pose.y - 5.3815`**, **`z = 0.0`** so that turtlesim coordinates are centered in the **`odom`** frame (values may be parameterized but default to these constants). 
- **Propagate velocity**:
  - Copy `linear_velocity` to **`odom.twist.twist.linear.x`** and `angular_velocity` to **`odom.twist.twist.angular.z`** to provide users (e.g., visualizers, controllers) with instantaneous motion. 
- **Per-callback publication**:
  - On **every** `/turtleX/pose` message, publish the corresponding `/odomX` and send the matching TF transform **without rate-skipping**. 
- **Frame and naming consistency**:
  - Use **`"odom"`** as the global frame for both turtles; ensure **`child_frame_id`** matches the turtle name and that Odometry and TF are consistent.
- **Node must able to run by `ros2 run` in Terminal**
---
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