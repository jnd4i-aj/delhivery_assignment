The repository does not contain detailed usage instructions in the `README.md` file. However, the `setup.py` files, `CMakeLists.txt`, and launch files provide useful hints. Here's a summarized guide to help you get started with this project:

### 1. **Install Prerequisites**
   - Ensure you have Python installed.
   - Install ROS 2 (Robot Operating System) as this project appears to be designed for ROS 2.

### 2. **Set Up the Environment**
   - Clone the repository:
     ```bash
     git clone https://github.com/jnd4i-aj/delhivery_assignment.git
     cd delhivery_assignment
     ```
   - Create and activate a virtual environment:
     ```bash
     python3 -m venv env
     source env/bin/activate
     ```
   - Install dependencies:
     ```bash
     pip install -r requirements.txt
     ```

### 3. **ROS 2 Integration**
   - Ensure ROS 2 is sourced:
     ```bash
     source /opt/ros/<distro>/setup.bash
     ```
   - Build the ROS 2 workspace:
     ```bash
     colcon build
     ```
   - Source the workspace:
     ```bash
     source install/setup.bash
     ```

### 4. **Launch the Nodes**
   - Use the provided launch file to bring up all nodes:
     ```bash
     ros2 launch robot_navigator all_nodes_bringup.launch.py
     ```
   - This launch configuration will initialize nodes like `navigation_node`, `status_node`, `task_manager_node`, and `central_logging_node`.

### 5. **Project Structure**
   - The project consists of several packages:
     - `robot_navigator`: Contains navigation logic and nodes.
     - `task_manager`: Manages tasks for the robot.
     - `central_logger`: Logs centralized data.
     - `path_planning`: Likely includes path planning algorithms.
     - `utils`: Provides shared utilities for the project.

### 6. **Further Customization**
   - Modify configuration files (e.g., `robot_config_params.yaml`) as needed for your environment.

If you need detailed steps about any part of the setup, let me know!
