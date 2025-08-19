# Multi-Sensor Fusion

![ROS2](https://img.shields.io/badge/ROS2-Humble-blue.svg)  ![Build](https://img.shields.io/badge/build-passing-brightgreen.svg)  

---

## Overview  
This project demonstrates **sensor fusion** between a **2D LiDAR** and an **RGB camera** using **ROS2 + Python**.  
It synchronizes LiDAR scans with camera images, applies extrinsic calibration using **tf2**, and projects LiDAR points into the image plane for robust multi-sensor perception.  

<p align="center">
  <img src="[https://github.com/user-attachments/assets/0047bef1-f741-4480-a9f4-ecb6210c86f1](https://pub.mdpi-res.com/symmetry/symmetry-12-00324/article_deploy/html/images/symmetry-12-00324-g001.png?1583498614)" alt="sensor fusion" height="500">
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
