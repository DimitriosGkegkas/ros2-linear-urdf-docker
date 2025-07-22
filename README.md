# 🐳 ROS2 Docker Environment (Humble Hawksbill)

This project sets up a minimal development environment for **ROS2 Humble** using Docker and `docker-compose`.

It includes:
- ROS2 Humble (`ros-humble-desktop`)
- Common libraries: `Eigen`, `yaml-cpp`, `rviz_visual_tools`
- Pre-configured workspace mount at `/ros2_ws/src`
- Automatic sourcing of the ROS2 environment

---

## 🛠️ Setup

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

## 🧪 Testing with ROS2 Demo Nodes

With two terminals open inside the container:

### **Terminal 1: Start the Listener (Python)**
```bash
ros2 run demo_nodes_py listener
```

### **Terminal 2: Start the Talker (C++)**
```bash
ros2 run demo_nodes_cpp talker
```

You should see the listener receive messages from the talker, verifying that the ROS2 setup works correctly.

---

## 🛑 Stopping the Environment

To stop and remove the running container:

```bash
docker compose down
```

---

## 📁 Project Structure

```
.
├── Dockerfile
├── docker-compose.yml
├── README.md        <-- You're here!
└── src/             <-- Your ROS2 packages live here
```

---

## ✅ Notes

- The ROS2 environment is sourced automatically via `/etc/bash.bashrc`
- The workspace directory `/ros2_ws/src` is mounted from your local `./src` folder
- You can build ROS2 packages inside the container using:

```bash
cd /ros2_ws
colcon build
source install/setup.bash
```

---

### Step 2: Linear Algebra ROS2 Service

This stage introduces a ROS2 service and two C++ nodes:

- A **server node** that solves a least-squares system using Eigen and applies a random rotation and translation.
- A **client node** that loads a matrix `A` and vector `b` from a YAML file, sends them to the server, receives the transformed solution, inverts the transformation, and publishes the result.

#### Build the Package

Inside the container:

```bash
docker exec -it ros2_container bash
cd /ros2_ws
colcon build
source install/setup.bash
```

#### Run the Server Node

```bash
ros2 run linear_algebra_service server
```

#### Run the Client Node (in a second terminal)

```bash
docker exec -it ros2_container bash
cd /ros2_ws
source install/setup.bash
ros2 run linear_algebra_service client --ros-args --params-file ./src/linear_algebra_service/config/matrix.yaml 
```

#### Parameters

The client reads a YAML file containing:

```yaml
/client:
  ros__parameters:
    A: [1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0,
        1.0, 1.0, 1.0]
    b: [1.0, 2.0, 3.0, 6.0]
```

#### Notes

- The server subscribes to the client's topic and uses a thread with a condition variable to log received messages.
- Place the service interface in its own package (`linear_algebra_interfaces`) and nodes in a shared package (`linear_algebra_service`).
  
  
---

Happy hacking with ROS2! 🛠️🐢
