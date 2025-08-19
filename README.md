# Multi-Sensor Fusion

![ROS2](https://img.shields.io/badge/ROS2-Humble-blue.svg)  ![Build](https://img.shields.io/badge/build-passing-brightgreen.svg)  

---

## Overview  
This project demonstrates **sensor fusion** between a **2D LiDAR** and an **RGB camera** using **ROS2 + Python**.  
It synchronizes LiDAR scans with camera images, applies extrinsic calibration using **tf2**, and projects LiDAR points into the image plane for robust multi-sensor perception.  

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
