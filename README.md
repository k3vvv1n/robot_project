# **Autonomous Ball Tracking Robot**  
**ROS2-based Real-Time Object Tracking System**  

![Robot Demo](media/image1.png)  

## **Project Overview**  
This project implements an **autonomous ball-tracking robot** using:  
- **Raspberry Pi 5** (ROS2 Humble)  
- **USB Webcam** (OpenCV for HSV-based ball detection)  
- **PID Control** for smooth pursuit  
- **4WD Rover Platform** with pan-tilt servo  

**Key Features:**  
✔ Real-time color-based ball tracking  
✔ Modular ROS2 architecture (video capture → detection → PID control)  
✔ Differential drive with zero-turn radius  
✔ Lightweight (~2.25 kg) and low-cost design  

---

## **Hardware Setup**  
| Component               | Specification                          |  
|-------------------------|----------------------------------------|  
| **Raspberry Pi 5**      | ROS2 Humble, Debian Bookworm           |  
| **Camera**              | 1080p @ 30 FPS, 160° FOV               |  
| **Motors**              | 4× DC (12V, 200 RPM), L298N driver     |  
| **Battery**             | 3S Li-ion (12V), 18650 cells           |  
| **Chassis**             | Acrylic 4WD (230×252×254 mm)           |  

![Hardware Diagram](media/image2.png)  

---

## **Software Architecture**  
### **ROS2 Nodes**  
1. **`video_capture_node`**  
   - Publishes raw frames → `/camera/image_raw`  

2. **`ball_detection_node`**  
   - Subscribes to `/camera/image_raw`  
   - Uses **OpenCV** for:  
     - HSV color thresholding  
     - Contour detection & centroid calculation  
   - Publishes annotated frames → `/ball_detection/output_image`  

3. **`pid_controller_node`**  
   - Subscribes to `/ball_detection/output_image`  
   - Implements **PID control** for motion correction  
   - Publishes velocities → `/cmd_vel`  

![ROS2 Nodes](media/image5.png)  

---

## **Algorithm Workflow**  
1. **Detect Ball** (HSV filtering + circularity check)  
2. **Calculate Errors**:  
   - X/Y offset from frame center  
   - Distance (from apparent ball size)  
3. **PID Control**: Adjusts wheel speeds to minimize errors  
4. **Pan-Tilt Servo**: Keeps ball centered in FOV  

![Flowchart](media/image7.png)  

---

## **Performance Metrics**  
| Scenario  | Initial Error (px) | Final Error (px) | Convergence Time (s) |  
|-----------|--------------------|------------------|----------------------|  
| **Red**   | X:5.2, Y:-4.8      | X:1.2, Y:-0.9    | 20                   |  
| **Blue**  | X:3.1, Y:-2.5      | X:0.3, Y:-0.1    | 20                   |  
| **Green** | X:6.8, Y:-5.2      | X:1.5, Y:-1.1    | 20                   |  

![Error Plot](media/image9.png)  

---

## **Installation & Usage**  
```bash  
# Clone repository  
git clone https://github.com/k3vvv1n/robot_project.git  

# Build ROS2 workspace  
colcon build  

# Run nodes  
ros2 launch ball_tracking ball_tracker.launch.py  
```  

**Dependencies**:  
- ROS2 Humble  
- OpenCV 4.x  
- `rclpy`, `sensor_msgs`, `geometry_msgs`  

---

## **Future Work**  
- [ ] Obstacle avoidance (ultrasonic/LiDAR)  
- [ ] Multi-ball tracking  
- [ ] Adaptive PID tuning  

---

## **References**  
1. OpenCV Documentation: [https://docs.opencv.org](https://docs.opencv.org)  
2. ROS2 Humble Docs: [https://docs.ros.org](https://docs.ros.org)  
3. Åström & Hägglund (2006). *Advanced PID Control*.  

---

**Developed by**:  
Miramzhanuly Arsen, Jumashev Tair, Lashyn Bektas  
**Kazakh-British Technical University**, 2025  
[Project GitHub](https://github.com/k3vvv1n/robot_project)  

--- 
![Demo GIF](media/demo.gif) *Live tracking demo*
