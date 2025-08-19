# Multi-Sensor Fusion

![ROS2](https://img.shields.io/badge/ROS2-Humble-blue.svg)  ![Build](https://img.shields.io/badge/build-passing-brightgreen.svg)  

---

## Overview  
This project demonstrates **sensor fusion** between a **2D LiDAR** and an **RGB camera** using **ROS2 + Python**.  
It synchronizes LiDAR scans with camera images, applies extrinsic calibration using **tf2**, and projects LiDAR points into the image plane for robust multi-sensor perception.  

<p align="center">
  <img src="https://pub.mdpi-res.com/symmetry/symmetry-12-00324/article_deploy/html/images/symmetry-12-00324-g001.png?1583498614" alt="sensor fusion" height="500">
</p>
<p align="center">
  Sensor Fusion
</p>

---

## Features  
- Subscribe to `/scan` (LiDAR) and `/camera/image_raw` (Camera)  
- Synchronize data using `message_filters`  
- Apply **extrinsics** (LiDAR → Camera) via tf2  
- Overlay LiDAR points on images  
- Publish fused outputs:  
  - `/fused_image` → LiDAR + Camera overlay  
  - `/camera/pointcloud2` → LiDAR projected in camera frame  

---

## 📂 Project Structure  
<p align="center">
  <img src="https://github.com/ipranaysatija/Multi-Sensor-Fusion/blob/main/Screenshot%202025-08-19%20190212.png?raw=true" alt="sensor fusion" height="500">
</p>

---

## Installation  

### Prerequisites  
```bash
# ROS2 (Humble/Foxy recommended)
# Python 3.8+
sudo apt install ros-${ROS_DISTRO}-cv-bridge ros-${ROS_DISTRO}-tf2-ros \
                 ros-${ROS_DISTRO}-image-transport ros-${ROS_DISTRO}-pcl-ros
pip install opencv-python numpy

```
### Clone and Build
```bash
cd ~/ros2_ws/src
git clone https://github.com/ipranaysatija/Multi-Sensor-Fusion.git
cd ~/ros2_ws
colcon build
source install/setup.bash
```
### Run
```bash
ros2 launch multi_sensor_fusion fusion_launch.py
```

---

## Future Work  

- Extend to **3D LiDAR**  
- Add **object detection + bounding box fusion**  
- Integrate **tracking (Kalman/Particle Filters)**  

---

## Contributing
1. Fork the repo  
2. Create a feature branch → git checkout -b feature-name  
3. Commit changes → git commit -m "Add new feature"  
4. Push to branch → git push origin feature-name  
5. Create a Pull Request  
