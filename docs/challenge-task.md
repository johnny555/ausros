# TurtleBot3 Challenge Task

## 1. Launch Bringup on the TurtleBot3

From the Ubuntu laptop:

```bash
ssh ubuntu@{IP_ADDRESS_OF_RASPBERRY_PI}
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch turtlebot3_bringup robot.launch
```

## 2. (Optional) Launch Camera Node

If you are using the camera for your task, launch the camera node on the TurtleBot3:

```bash
ssh ubuntu@{IP_ADDRESS_OF_RASPBERRY_PI}
ros2 run v4l2_camera v4l2_camera_node
```

**Note:** Check if images need to be compressed and/or change image resolution settings on the camera. See the note at the bottom of [this page](https://emanual.robotis.com/docs/en/platform/turtlebot3/sbc_setup/).

---

# TurtleBot3 Navigation

The `turtlebot3_navigation` package provides an interface to the ROS2 Nav2 library ([docs](https://docs.nav2.org)), which has implementations for mapping, localisation, SLAM, and waypoint navigation.

To run SLAM and waypoint navigation:

```bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py slam:=True
```

If running in simulation, include the flag `use_sim_time:=True`.

---

## 3.4 Waypoint Publishing Node

### 1. Create a New Workspace

```bash
cd ~
mkdir -p ausros_ws/src
cd ausros_ws
colcon build
```

### 2. Source the Workspace

```bash
source ~/ausros_ws/install/setup.bash
```

You can add this line to your `~/.bashrc` to source the workspace automatically:

```bash
echo 'source ~/ausros_ws/install/setup.bash' >> ~/.bashrc
```

### 3. Create the `waypoint_commander` Package

```bash
cd ~/ausros_ws/src
ros2 pkg create --build-type ament_python waypoint_commander --dependencies rclpy,nav2_msgs,geometry_msgs
```

You can also manually update the package’s execution time dependencies in `package.xml`:

```xml
<exec_depend>rclpy</exec_depend>
<exec_depend>nav2_msgs</exec_depend>
<exec_depend>geometry_msgs</exec_depend>
```

### 4. Create the `waypoint_cycler.py` Node

```bash
cd ~/ausros_ws/src/waypoint_commander/waypoint_commander
touch waypoint_cycler.py
```

### 5. Basic Node Structure

Open `waypoint_cycler.py` and start with:

```python
import rclpy
from rclpy.node import Node
from nav2_msgs.msg import BehaviorTreeLog
from geometry_msgs.msg import PoseStamped

def main(args=None):
    rclpy.init(args=args)
    waypoint_cmder = WaypointCycler()
    rclpy.spin(waypoint_cmder)

if __name__ == '__main__':
    main()
```

### 6. Create the `WaypointCycler` Node Class

Initialize with:

- A subscriber to the `behavior_tree_log` topic
- A publisher to the `goal_pose` topic
- A waypoint counter
- A list of waypoints

```python
class WaypointCycler(Node):
    def __init__(self):
        super().__init__('waypoint_cycler')
        self.subscription = self.create_subscription(
            BehaviorTreeLog,
            'behavior_tree_log',
            self.bt_log_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.publisher_ = self.create_publisher(
            PoseStamped,
            'goal_pose',
            10)

        self.waypoint_counter = 0

        # Example waypoints (adjust as needed)
        p0 = PoseStamped()
        p0.header.frame_id = 'map'
        p0.pose.position.x = 1.7
        p0.pose.position.y = -0.5
        p0.pose.orientation.w = 1.0

        p1 = PoseStamped()
        p1.header.frame_id = 'map'
        p1.pose.position.x = -0.6
        p1.pose.position.y = 1.8
        p1.pose.orientation.w = 1.0

        self.waypoints = [p0, p1]
```

### 7. Behavior Tree Log Callback

```python
def bt_log_callback(self, msg: BehaviorTreeLog):
    for event in msg.event_log:
        if event.node_name == 'NavigateRecovery' and event.current_status == 'IDLE':
            self.send_waypoint()
```

### 8. Send Waypoint Function

```python
def send_waypoint(self):
    self.waypoint_counter += 1
    if self.waypoint_counter % 2:
        self.publisher_.publish(self.waypoints[1])
    else:
        self.publisher_.publish(self.waypoints[0])
```

### 9. Update `setup.py` Entry Points

In your top-level `waypoint_commander/setup.py`, add the following to the `entry_points` list (after line 22, inside the square brackets):

```python
'waypoint_cycler = waypoint_commander.waypoint_cycler:main',
```

### 10. Build and Run

Build your package:

```bash
colcon build --symlink-install --packages-select waypoint_commander
```

Run your node:

```bash
ros2 run waypoint_commander waypoint_cycler
```

**Note:** To kick off `waypoint_cycler`, you need to first send a manual waypoint to the TurtleBot3 (either in RViz or through the command line). Once it reaches the waypoint, the cycler will take over.

```python
    if self.waypoint_counter % 2:
        self.publisher_.publish(self.waypoints[1])
    else:
        self.publisher_.publish(self.waypoints[0])
```


9. Save your file and open the setup.py file in your top-level waypoint_commander folder. In order for
ROS to know about our new node, we need to include the following after line 22 (inside the square
brackets):

`'waypoint_cycler = waypoint_commander.waypoint_cycler:main',`

10. In a terminal, colcon build your package from your workspace folder.


`colcon build --symlink-install --packages-select waypoint_commander`


1.  Run your node!

`ros2 run waypoint_commander waypoint_cycler`

To kick off waypoint_cycler you need to first send a manual waypoint to the TurtleBot3 (either in
RViz or through the command line). Once it reaches
