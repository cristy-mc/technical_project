# 🏥 Autonomous Hospital Logistics System (Multi-Robot ROS 2 & Gazebo)

An autonomous logistics simulation project built using **ROS 2** and **Gazebo**, designed to automate the transport of medical instruments (such as scalpels, scissors, and forceps) within a hospital environment. The system relies on a multi-robot cooperative setup consisting of two fixed KUKA iiwa manipulator arms and one mobile rover (`fra2mo`).

---

## ⚙️ System Architecture & Components

1. **`iiwa2` (Storage Station):** 
   - A fixed KUKA iiwa arm located at the storage table.
   - Equipped with custom pick-and-place plugins (`attach/detach`) to pick up medical tools and load them onto the rover.
2. **`fra2mo` (Mobile Shuttle):** 
   - A differential drive mobile robot equipped with a **LIDAR** sensor and an **RGB camera**.
   - Responsible for autonomous point-to-point transit (shuttling) between the storage and patient areas using the **Nav2** stack and computer vision.
3. **`iiwa1` (Patient Bed Station):** 
   - A second fixed KUKA iiwa arm located next to the patient's bed table (`bedTable`).
   - Receives the instruments from the rover and performs the final placement.


## 🚀 How to Run the Simulation

To launch the complete robotic system, open a new terminal for each of the following commands (remember to source your ROS 2 workspace in every terminal, e.g., `source install/setup.bash`):

**1. Launch the Gazebo Environment (World, 3 Robots, and Patient):**
```
ros2 launch iiwa_description iiwa_world.launch.py
``` 
**2. Launch the Rover's Navigation Stack (Nav2 & AMCL):**
```
ros2 launch ros2_fra2mo fra2mo_navigation.launch.py
```

**3. Start the Robot Arm Controllers:**
```
ros2 run hospital iiwa1_pick_node

ros2 run hospital iiwa2_pick_kdl_node
```

**4. Start the Vision and Docking Nodes:**
```
ros2 run hospital object_detector.py
ros2 run hospital aruco_docking_node.py
```

**5. Start the Mission Coordinator:**
```
ros2 run hospital task_coordinator.py
```

**6. Trigger the Mission (User Request):**
```
ros2 run hospital user_request_node.py
```

---

## 🗺️ Mapping & Localization
Prior to the autonomous mission, the rover explored the OR to generate a 2D occupancy grid map using SLAM. 
This map is utilized by the **AMCL** node for real-time localization and by the **Nav2 Costmaps** to ensure safe path planning around static obstacles.

---

## 📋 Mission Workflow
The entire mission is asynchronously orchestrated by a central Python `task_coordinator` node:
1. **Surgeon Request:** A specific tool is requested via the `/object_request` topic.
2. **Rover Deployment:** Nav2 guides the rover to the iiwa2 landing pad.
3. **Pick & Load:** The iiwa2 arm detects the tool using Computer Vision, picks it up, and places it on the rover's tray.
4. **Transit:** The rover moves autonomously to the iiwa1 landing pad.
5. **Final Delivery:** The iiwa1 arm picks the tool from the rover and securely places it on the surgical table.

---


