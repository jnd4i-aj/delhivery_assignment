## **Project:** **Delhivery Assignment**

## Problem Statement

Design a ROS-based architecture simulating an autonomous warehouse robot system. The focus is on
proper ROS communication, modularity, and system observability through logging and publishing logs as
messages for monitoring purposes.


## Overview

This is a ROS 2-based project for simulating, assigning, and monitoring delivery tasks using distributed task manager and navigation nodes. The repository includes robot navigation, task allocation, and central logging to facilitate research and development in multi-robot or automated delivery workflows.

---

## Workflow

1. **Task Assignment (Task Manager Node):**
   - The Task Manager receives new delivery tasks (pickup and drop nodes).
   - It manages and assigns tasks to robots, plans paths using a map, and communicates tasks via ROS 2 topics/services.

2. **Navigation Execution (Robot Navigator Node):**
   - Robot Navigator subscribes to assigned tasks, computes and follows the path for pickup and delivery, and publishes its status.

3. **Logging & Monitoring:**
   - All task and navigation events are published to a central logger for monitoring and debugging.
   - Logs can be tailed, filtered, or exported for analysis.

---

## Repository Structure

- **task_manager/**  
  - Implements the Task Manager Node for task allocation and path planning.
  - Contains ROS 2 node, service, and publisher/subscriber logic.

- **robot_navigator/**  
  - Implements the Navigation Node that executes assigned tasks and reports status.
  - Contains path execution logic, status publishing, and error handling.

- **interfaces/**  
  - Contains ROS 2 message and service definitions for task queues, navigation data, robot status, and logging.

- **central_logger/**  
  - Publishes and aggregates logs from all nodes.

- **utils/**  
  - Utility modules, e.g., file system helpers and path planning.

---

## Main Features

- **Task Assignment**: Allocates delivery tasks using ROS 2 services, parses and validates task IDs, and manages a queue.
- **Path Planning**: Computes optimal paths for pickup and delivery using a map loaded from YAML.
- **Robot Navigation**: Executes the planned path, handles pickup/drop, and reports status back to the system.
- **Centralized Logging**: All nodes publish log messages to a central logger for observability and debugging.
- **ROS 2 Integration**: Uses ROS 2 topics and services for modular, distributed operation.

---

## Getting Started

> **Prerequisites:**  
> - ROS 2 (Foxy or later recommended)  
> - Python 3.8+  
> - (Optional) CMake for building custom interface messages

### Clone the Repository

```bash
git clone https://github.com/jnd4i-aj/delhivery_assignment.git
cd delhivery_assignment
```

### Build the Workspace

```bash
# If using ROS 2 colcon workspace:
colcon build
source install/setup.bash
```

Or, for Python-only development:

```bash
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

---

### Run Task Manager Node

```bash
ros2 run task_manager task_manager_node
```

### Run Robot Navigator Node

```bash
ros2 run robot_navigator navigation_node
```

- Both nodes will communicate via ROS 2 topics/services.
- Logs will be published and can be monitored as described below.

---

## CLI Commands

### Task Management

- **Start Delivery Task (via service call):**
  ```bash
  ros2 service call /start_delivery interfaces/srv/AllocateTask "{task_id: 'P1D5'}"
  ```
  - Assigns a task from pickup node 1 to drop node 5.

### Monitoring & Logs

- **Tail All Logs**
  ```bash
  tail -f logs/app.log
  ```

- **Show Only Errors**
  ```bash
  grep ERROR logs/app.log
  ```

- **Export Logs**
  ```bash
  cp logs/app.log exported_logs.txt
  ```

---

## File Highlights

- `task_manager/task_manager/task_manager_node.py`: Implements the Task Manager ROS 2 node.
- `robot_navigator/robot_navigator/navigation_node.py`: Executes task navigation and status publishing.
- `interfaces/msg/` and `interfaces/srv/`: Custom message and service definitions for inter-node communication.
- `central_logger/log_publisher.py`: Publishes logs from all nodes.

---

## License

This project currently does not specify a license. If you plan to use it beyond personal or academic purposes, please clarify with the repository owner.

---

## Author

Name: AMAN JINDAL  
GitHub: [jnd4i-aj](https://github.com/jnd4i-aj)

---

Would you like this README.md written directly to your repository? Or do you want further details on any section (e.g., specific message/service types, example logs, or advanced usage)?
