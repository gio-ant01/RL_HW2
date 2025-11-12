# RL_HW2
This is the repository for RoboticsLab HW2. Link to the playlist containing our demo videos on Youtube: 
https://youtube.com/playlist?list=PLIDEFmFRzNxw_BD0JExKTiWNJHv8fP9Y-&si=qAr_fYw2-pXmJIfy

## 🔨 Installation

### Clone Required Repositories

```bash
cd ~/ros2_ws/src
```

### Install Required Packages

```bash
# Core ROS 2 packages
sudo apt update
sudo apt install -y ros-humble-urdf-launch
sudo apt install -y ros-humble-controller-manager
sudo apt install -y ros-humble-ign-ros2-control
sudo apt install -y ros-humble-xacro
sudo apt install -y ros-humble-rqt-image-view
sudo apt install -y ros-humble-kdl-parser
sudo apt install -y ros-humble-joint-state-publisher
sudo apt install -y ros-humble-robot-state-publisher

# Gazebo bridge packages
sudo apt install -y ros-humble-ros-gz-bridge
sudo apt install -y ros-humble-ros-gz-sim

# Additional dependencies
sudo apt install -y libeigen3-dev
sudo apt install -y liborocos-kdl-dev
```

### Build the Workspace

```bash
cd ~/ros2_ws/src
colcon build
source install/setup.bash
```

## ✅ Usage

---

## **1. Basic Trajectory Control (1.a)**

### Launch Simulation with Velocity Controller

**Terminal 1** - Start simulation:
```bash
ros2 launch iiwa_bringup iiwa.launch.py command_interface:="velocity" robot_controller:="velocity_controller"
```

**Terminal 2** - Launch KDL control node:
```bash
ros2 launch ros2_kdl_package kdl_launch.py
```

The robot will execute a linear trajectory using parameters defined in `config/kdl_params.yaml`.

```

---

## **2. Velocity Control with Null Space Optimization (1.b)**

This controller adds joint limit avoidance using null space projection.

### Standard Velocity Controller

**Terminal 1**:
```bash
ros2 launch iiwa_bringup iiwa.launch.py command_interface:="velocity" robot_controller:="velocity_controller"
```

**Terminal 2**:
```bash
ros2 launch ros2_kdl_package kdl_launch.py ctrl:=velocity_ctrl
```

### Null Space Velocity Controller

**Terminal 1**:
```bash
ros2 launch iiwa_bringup iiwa.launch.py command_interface:="velocity" robot_controller:="velocity_controller"
```

**Terminal 2**:
```bash
ros2 launch ros2_kdl_package kdl_launch.py ctrl:=velocity_ctrl_null
```

### Data Logging and Comparison

To record joint trajectories for analysis:

**Terminal 1** - Simulation:
```bash
ros2 launch iiwa_bringup iiwa.launch.py command_interface:="velocity" robot_controller:="velocity_controller"
```

**Terminal 2** - Data logger:
```bash
cd ~/ros2_ws/src/ros2_kdl_package/scripts
python3 data_logger.py --ros-args -p ctrl_type:=velocity_ctrl
# or
python3 data_logger.py --ros-args -p ctrl_type:=velocity_ctrl_null
```

**Terminal 3** - Controller:
```bash
ros2 launch ros2_kdl_package kdl_launch.py ctrl:=velocity_ctrl
# or
ros2 launch ros2_kdl_package kdl_launch.py ctrl:=velocity_ctrl_null
```

After trajectory completion, press `Ctrl+C` to save the CSV file.
(Our CSV files are in the following folder: /ros2_kdl_package/scripts).

### Analyze Data in MATLAB

```matlab
cd ~/ros2_ws/src/ros2_kdl_package/scripts
% Edit plot_comparison.m to match your CSV filenames
plot_comparison
```

---

## **3. Action-Based Trajectory Execution (1.c)**

The action server allows asynchronous trajectory execution with real-time feedback.

### Launch Action Server and Client

**Terminal 1** - Simulation:
```bash
ros2 launch iiwa_bringup iiwa.launch.py command_interface:="velocity" robot_controller:="velocity_controller"
```

**Terminal 2** - Action server:
```bash
ros2 run ros2_kdl_package ros2_kdl_action_server
```

**Terminal 3** - Action client (with launch file):
```bash
ros2 launch ros2_kdl_package action_client_launch.py
```


### Configure Action Parameters

Edit `config/action_client_params.yaml`:
```yaml
linear_trajectory_action_client:
  ros__parameters:
    traj_duration: 2.0
    acc_duration: 0.5
    end_position_x: 0.5
    end_position_y: 0.3
    end_position_z: 1.2
    traj_type: "linear"
    s_type: "trapezoidal"
    lambda: 1.2
```

---

## **4. ArUco Marker Spawn (2.a)**

### Launch Simulation with ArUco Marker

**Terminal 1** - Simulation with camera and ArUco world:
```bash
ros2 launch iiwa_bringup iiwa_aruco_launch.py
```
**Terminal 2** - Camera output visualization:
```bash
ros2 run rqt_image_view rqt_image_view /iiwa/camera
```

---

## **5. ArUco Marker Detection (2.b)**

### Launch Simulation with ArUco Marker

**Terminal 1** - Simulation with camera and ArUco world:
```bash
ros2 launch iiwa_bringup iiwa_aruco_2b_launch.py command_interface:="velocity" robot_controller:="velocity_controller"
```

**Terminal 2** - Subscribe to ArUco pose:
```bash
ros2 run iiwa_bringup aruco_pose_subscriber.py
```

**Terminal 3** - Show the topic:
```bash
ros2 topic echo /aruco_single/pose
```

**Terminal 4** - View detection result:
```bash
ros2 run rqt_image_view rqt_image_view /aruco_single/result
```

---

## **6. Vision-Based Control (2.b)**

Visual servoing controller that tracks the ArUco marker using a look-at-point control law.

**Terminal 1** - Simulation:
```bash
ros2 launch iiwa_bringup iiwa_aruco_2b_launch.py command_interface:="velocity" robot_controller:="velocity_controller"
```

**Terminal 2** - Vision control node:
```bash
ros2 launch ros2_kdl_package vision_control.launch.py K_vision:=1.0 lambda:=100.0
```

**Terminal 3** - Visualize detection:
```bash
ros2 run rqt_image_view rqt_image_view /aruco_single/result
```

**Terminal 4** - Monitor velocity commands:
```bash
ros2 topic echo /velocity_controller/commands
```

### Test by Moving the Marker

You can manually move the ArUco marker in Gazebo using the GUI to observe the robot's tracking behavior.

---

## **7. Dynamic Marker Repositioning (2.c)**

### Launch with Service Bridge

**Terminal 1** - Simulation with service bridge:
```bash
ros2 launch iiwa_bringup iiwa_aruco_2c_launch.py
```
**Terminal 2** - Service call:
```bash
ros2 service call /world/default/set_pose ros_gz_interfaces/srv/SetEntityPose "
entity:
  name: 'aruco_tag'
  type: 0
pose:
  position:
    x: 1.0
    y: 2.0
    z: 0.5
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0
"

```

### Verify Service Availability

```bash
ros2 service list | grep set_pose
```

Expected output:
```
/world/default/set_pose
```

### Check Service Type

```bash
ros2 service type /world/default/set_pose
```

Expected output:
```
ros_gz_interfaces/srv/SetEntityPose
```


---

## 📄 License

Apache-2.0

---

## 🔗 References

- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/)
- [Gazebo Ignition](https://gazebosim.org/)
- [ArUco ROS](https://github.com/pal-robotics/aruco_ros)
- [Orocos KDL](https://www.orocos.org/kdl.html)
