# Phone Visualizer with ROS 2

This project lets you visualize your phone in **RViz2** using ROS 2 nodes, giving you access to the phone's **Rotation Vector Sensor** without buying a physical gyroscope or creating a custom app to extract rotation vectors from angular speed.

## Requirements

- ROS 2
- [`phonesensors` Python package](https://github.com/nup002/PhoneSensors.git)
- [SensorStreamer Android App](https://apkpure.com/sensorstreamer/cz.honzamrazek.sensorstreamer/downloadApplication)

## Setup

### 1. Configure the SensorStreamer App

1. Open **Manage Connections** → **Add New Connection**  
   - Choose a **port greater than 1024** (lower ports are restricted by Android).
2. Open **Manage Packets** → Add a new packet → Enable **Rotation Vector Sensor**.


---

### 2. Configure the ROS 2 Package
- Get your phone’s IP address from: **Settings → About Phone → IP Address**.
Edit the YAML file in:

**Path:** `src/vis_phone/config/phone_orient_params.yaml`
```yaml
/phonesensor_node:
  ros__parameters:
    IP_address: "192.168.0.75"
    port: 8080
```

---

### 3. Install the `phonesensors` Package

**Using pip:**
```bash
pip install phonesensors
```

**Using uv + venv:**
```bash
uv pip install -r requirements.txt
source /home/<username>/.venv/bin/activate
```

Or install all dependencies in any virtual environment and activate it.

---

### 4. Build and Source the ROS 2 Packages

From the repository root (`.../repo-name/`):

```bash
colcon build
source ./install/setup.bash
```

---

### 5. Connect Devices

Ensure your **phone** and **PC** are connected to the **same Wi-Fi network**.

---

### 6. Launch the Visualizer

Run:
```bash
ros2 launch vis_phone display.launch.py
```

Then press **Start** in the SensorStreamer app to begin streaming data.

---

> **Note:** Don’t forget to source your ROS 2 environment before running:
> ```bash
> source /opt/ros/<ros-distro>/setup.bash
> ```