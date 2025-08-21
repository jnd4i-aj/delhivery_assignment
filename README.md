## **Project:** **Delhivery Assignment**

## Problem Statement

Design a ROS-based architecture simulating an autonomous warehouse robot system. The focus is on
proper ROS communication, modularity, and system observability through logging and publishing logs as
messages for monitoring purposes.


## Overview

This is a ROS 2-based project for simulating, assigning, and monitoring delivery tasks using a distributed task manager and navigation nodes. The repository includes robot navigation, task allocation, and central logging for an autonomous warehouse robot system for the automated delivery workflows.

---

## Workflow

1. **Task Assignment (Task Manager Node):**
   - The Task Manager receives new delivery tasks (pickup and drop nodes).
   - It manages and assigns tasks to robots, plans paths using a map, and communicates tasks via ROS 2 topics/services.

2. **Navigation Execution (Robot Navigator Node):**
   - Navigation Node subscribes to assigned tasks, computes and follows the path for pickup and delivery, and updates the Robot Data.
   - Robot Status Node subscribes to _/robot_status_ topic, and publishes and logs its status.

3. **Logging & Monitoring:**
   - Key log messages from the **_task_manager_node_, _navigation_node_**, and **_status_node_** are published to a **_central logger node_** for monitoring and debugging.
   - All the logs from all nodes are saved in their respective _.log_ files.
   - Logs can be tailed, filtered, or exported for analysis.

---

## Repository Structure

- **Package**: **task_manager/**  
  - Implements the Task Manager Node for task allocation and path planning.
  - Contains ROS 2 node, service, and publisher/subscriber logic.

- **Package**: **robot_navigator/**  
  - Implements the Navigation Node and Status Node that execute assigned tasks and publish the status.
  - Contains path execution logic, status publishing, and error handling.

- **Package**: **interfaces/**  
  - Contains ROS 2 message and service definitions for task queues (Topic Name: _/task_queue_), robot status (Topic Name: _/robot_status), logging (Topic Name: _/log), and navigation data.

- **Package**: **central_logger/**  
  - Publishes and aggregates logs from all nodes.
  - Save all the logs from all nodes to a **_.log_** file.

- **Package**: **utils/**  
  - Utility modules, e.g., file system helpers and a logger for saving logs in a file.

---

## Main Features

- **Task Assignment**: Allocates delivery tasks using ROS 2 services, parses and validates task IDs, computes optimal paths for pickup and delivery, and manages a queue.
- **Robot Navigation and Status**: Executes the planned path, handles pickup/drop, and reports status back to the system.
- **Centralized Logging**: All nodes publish log messages to a central logger for observability and debugging.
- **ROS 2 Integration**: Uses ROS 2 topics and services for modular, distributed operation.

---

## Getting Started

> **Prerequisites:**  
> - ROS 2 (Foxy or later recommended)  
> - Python 3.8+  
> - (Optional) CMake for building custom interface messages

### Clone the Repository

Clone this repository inside the **_ros2_ws/src_** folder.

```bash
git clone https://github.com/jnd4i-aj/delhivery_assignment.git
cd delhivery_assignment
```

### Build the Workspace

```bash
colcon build
source install/setup.bash
```

---

### Run Launch file

Run the launch file using the command below, which will automatically start all the nodes.

```bash
ros2 launch robot_navigator all_nodes_bringup.launch.py
```

- All nodes will communicate via ROS 2 topics/services.
- Logs will be published and can be monitored as described below.

---

## CLI Commands

### Task Management

- **Start Delivery Task (via service call):**
  ```bash
  ros2 service call /start_delivery interfaces/srv/AllocateTask "{task_id: 'P1D5'}"
  ```
  - **Assigns a task from pickup node 1 to drop node 5.**

### Monitoring & Observability

#### 1. Observe ROS2 Topics

- **List all active topics:**
  ```bash
  ros2 topic list
  ```
- **Inspect messages on a topic:**
  ```bash
  ros2 topic echo /topic_name
  ```
- **Get detailed info about a topic:**
  ```bash
  ros2 topic info /topic_name
  ```
  - **To see the data published on the /task_queue topic:**
  ```bash
  ros2 topic echo /task_queue
  ```
  - **To see the data published on the /robot_status topic:**
  ```bash
  ros2 topic echo /robot_status
  ```
  - **To see the data published on the /nav_data topic:**
  ```bash
  ros2 topic echo /nav_data
  ```
  - **To see the data published on the /log topic:**
  ```bash
  ros2 topic echo /log
  ```
  - **To see the data published on the /target_pose topic:**
  ```bash
  ros2 topic echo /target_pose
  ```

#### 2. List Services & Nodes

- **List all active nodes:**
  ```bash
  ros2 node list
  ```
- **List all available services:**
  ```bash
  ros2 service list
  ```
- **Get info about a node:**
  ```bash
  ros2 node info /node_name
  ```

#### 3. Interfaces

- **Show definition of the message:**
- 1. For Task Queue
  ```bash
  ros2 interface show interfaces/msg/TaskQueue
  ```
- 2. For Robot Status
  ```bash
  ros2 interface show interfaces/msg/RobotStatus
  ```
- 3. For All Logs
  ```bash
  ros2 interface show interfaces/msg/AllLogs
  ```
- 2. For Nav Data
  ```bash
  ros2 interface show interfaces/msg/NavData
  ```
- 2. For Target Pose
  ```bash
  ros2 interface show interfaces/msg/TargetPose
  ```

- **Show definition of the Start Delivery Service:**
  ```bash
  ros2 interface show interfaces/srv/AllocateTask
  ```

---
### Logs

- Logs are stored in the home directory of the computer, in the `service_logs` folder.
- Inside the `service_logs` folder, there will be a folder with the name `<todays_date>_all_logs`.
- Inside the `<todays_date>_all_logs` folder, there will be `.log` files with the name `<node_name>.log`

---

## File Highlights

- `task_manager/task_manager/task_manager_node.py`: Implements the Task Manager ROS 2 node.
- `robot_navigator/robot_navigator/navigation_node.py`: Executes task navigation and updates robot data.
- `robot_navigator/robot_navigator/status_node.py`: Receives the Robot's current Status, then publishes and logs it..
- `interfaces/msg/` and `interfaces/srv/`: Custom message and service definitions for inter-node communication.
- `central_logger/log_publisher.py`: Receives logs from all nodes by subscribing to topic _/log_ and saves to a _.log_ file.

---


## Author

Name: AMAN JINDAL  
GitHub: [jnd4i-aj](https://github.com/jnd4i-aj)

---
