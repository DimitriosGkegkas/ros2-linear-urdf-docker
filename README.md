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

Happy hacking with ROS2! 🛠️🐢
