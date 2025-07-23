# 🐳 ROS2 Dockerized Workspace (Humble Hawksbill)

This repository provides a minimal, containerized ROS2 Humble setup and includes several C++ and Python ROS2 packages for linear algebra processing and URDF-based robot visualization.

### 🧰 Features
- ROS2 Humble with `Eigen`, `yaml-cpp`, and `rviz_visual_tools`
- Docker-based setup with mounted workspace and auto-sourced environment

### 📦 Included Packages

- **`linear_algebra_service`**  
  Defines the custom service type `GetLeastSquares.srv` for solving least-squares problems.

- **`linear_algebra_nodes`**  
  Contains two C++ nodes:  
  - `client`: Loads matrix data from YAML, calls the service, inverts the transformation, and publishes the result  
  - `server`: Solves the least-squares problem, applies a random transform, and responds with transformed data

- **`ur20_display`**  
  - `joint_state_publisher` (C++): Publishes joint states, verifies TFs, and visualizes them in RViz  
  - `plotter` (Python): Subscribes to a trajectory topic and plots joint motion using `matplotlib`

!!!Y Yes the readme was formatted using ChatGPT for better look 🙃

## 🛠️ Setup

### ⚠️ Important: Clone with Submodules

Make sure to **clone the repository with submodules**:

```bash
git clone --recurse-submodules <repo-url>
```

### 1. Build the Docker Image

```bash
docker compose build
```

This will build the container with all dependencies installed.

---

### 2. Start the Container

```bash
docker compose up -d
```

Starts the container in detached mode. The `./src` directory is mounted into `/ros2_ws/src` inside the container.

---

### 3. Enter the Container Shell

Use this command in two separate terminal windows to simulate two ROS2 nodes:

```bash
docker exec -it ros2_container bash
```

---

### 4. Building ROS2 Packages

You can build ROS2 packages inside the container using:

```bash
cd /ros2_ws
colcon build
source install/setup.bash
```

---


## 🛑 Stopping the Environment

To stop and remove the running container:

```bash
docker compose down
```

#### ✅ Notes

- The ROS2 environment is sourced automatically via `/etc/bash.bashrc`
- The workspace directory `/ros2_ws/src` is mounted from your local `./src` folder

---

## Step 2: Linear Algebra ROS2 Service

This stage introduces a ROS2 service and two C++ nodes:

- A **server node** that solves a least-squares system using Eigen and applies a random rotation and translation.
- A **client node** that loads a matrix `A` and vector `b` from a YAML file, sends them to the server, receives the transformed solution, inverts the transformation, and publishes the result.

#### 1. Build the Package

Inside the container:

```bash
docker exec -it ros2_container bash
colcon build
```

#### Run the Server Node

```bash
docker exec -it ros2_container bash
source install/setup.bash
ros2 run linear_algebra_service server
```

#### Run the Client Node (in a second terminal)

```bash
docker exec -it ros2_container bash
source install/setup.bash
ros2 run linear_algebra_service client --ros-args --params-file ./src/linear_algebra_service/config/matrix.yaml 
```

#### Parameters

The client does not read a YAML file but gets the parameters from the command line or a yaml file

```yaml
/client:
  ros__parameters:
    A: [1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0,
        1.0, 1.0, 1.0]
    b: [1.0, 2.0, 3.0, 6.0]
```


## 🤖 Step 3: URDF Visualization and Robot State Control

This step launches a URDF visualization and simulates robot state behavior using RViz and a simple robot description package.

### 1. Build the Package

Inside the container:

```bash
docker exec -it ros2_container bash
colcon build
```

---

### 2. Launch URDF Visualization

To visualize the robot in RViz:

```bash
docker exec -it ros2_container bash
source install/setup.bash
ros2 launch ur20_display main.launch.py
```

### ✅ If You're Using a **Linux Host**

RViz and matplotlib plots should automatically display on your host screen Since Docker is configured correctly for GUI forwarding. However, you may need to allow the Docker container access to your X server:

```bash
xhost +local:root
```

> 🔒 **Important:** For security reasons, **always revoke access** after you're done:

```bash
xhost -local:root
```

For more details, refer to the official ROS Docker GUI tutorial:  
🔗 https://wiki.ros.org/docker/Tutorials/GUI

---

### 🍏 If You're Using **macOS** 

Docker containers on macOS cannot natively forward GUI applications like RViz. A simple and effective workaround is to use **Foxglove Studio**, a web-based visualization tool.

#### Steps:

## 🛠️ Setup

1. Install Foxglove Studio:  
   👉 https://foxglove.dev/download

2. Inside the container, start the ROSBridge WebSocket server:

   ```bash
   docker exec -it ros2_container bash
   source install/setup.bash
   ros2 launch rosbridge_server rosbridge_websocket_launch.xml
   ```

3. Open Foxglove Studio and connect to:

   ```
   ws://localhost:9090
   ```

   You can now view robot state and topic data interactively through the Foxglove interface.

> 🧠 **Note:** This solution works across platforms and is ideal for users who cannot run native GUI apps from Docker.
