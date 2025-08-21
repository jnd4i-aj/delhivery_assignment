## Getting Started

> **Prerequisites:**  
> - ROS2 (Foxy or later recommended)  
> - Python3

### Clone the Repository

```sh
git clone https://github.com/jnd4i-aj/delhivery_assignment.git
cd delhivery_assignment
```

### Build the Workspace

```sh
colcon build
source install/setup.bash
```

### Simulate Autonomous warehouse Robot System

Run the below command to launch the nodes:

```sh
ros2 launch robot_navigator all_nodes_bringup.launch.py
```

### Assign a task using the ROS2 Service

Open a new terminal, source your workspace, then:

```sh
ros2 service call /start_delivery interfaces/srv/AllocateTask "{task_id: P4D20}"
```


## License

This project currently does not specify a license. If you plan to use it beyond personal or academic purposes, please clarify with the repository owner.

---
