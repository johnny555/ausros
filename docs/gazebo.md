# Gazebo Harmonic

This project uses Gazebo harmonic. 

# Sensor Plugin 

This means that the world will need to include the sensors plugin as follows 

```xml
    <plugin
      filename="gz-sim-scene-broadcaster-system"
      name="gz::sim::systems::SceneBroadcaster">
    </plugin>
```

Yau can wrap the above in <gazebo> tags if added to a urdf.

## Per sensor plugin

You can add the sensor in the xacro, use the `<gazebo reference="link_name">` to specify which link in the robot will have the sensor attached. 

## Lidar

If you are adding a lidar, make sure to include this code to activate it per sensor: 

```xml

<gazebo reference="link_name">
    <sensor name='gpu_lidar' type='gpu_lidar'>
        <topic>lidar</topic>
        <update_rate>10</update_rate>
        <lidar>
        <scan>
            <horizontal>
            <samples>640</samples>
            <resolution>1</resolution>
            <min_angle>-1.396263</min_angle>
            <max_angle>1.396263</max_angle>
            </horizontal>
            <vertical>
            <samples>16</samples>
            <resolution>1</resolution>
            <min_angle>-0.261799</min_angle>
            <max_angle>0.261799</max_angle>
            </vertical>
        </scan>
        <range>
            <min>0.08</min>
            <max>10.0</max>
            <resolution>0.01</resolution>
        </range>
        </lidar>
        <alwaysOn>1</alwaysOn>
        <visualize>true</visualize>
    </sensor>
</gazebo>
```

# Depth Camera

```xml
<gazebo reference="optical_link">
    <sensor name="depth_camera" type="rgbd_camera">
        <update_rate>10</update_rate>
        <topic>realsense</topic>
        <camera name="camera">
            <optical_frame_id>realsense_link</optical_frame_id>

            <horizontal_fov>1.0472</horizontal_fov>

            <image>
                <width>640</width>
                <height>480</height>
                <format>R8G8B8</format>
            </image>
            <clip>
                <near>0.01</near>
                <far>300</far>
            </clip>
        </camera>
    </sensor>
</gazebo>
```

# Wheel Control 

You can add a diff drive control to your robots xacro, e.g.

```xml
<gazebo>
      <plugin
        filename="gz-sim-diff-drive-system"
        name="gz::sim::systems::DiffDrive">
        <left_joint>left_wheel_joint</left_joint>
        <right_joint>right_wheel_joint</right_joint>
        <wheel_separation>1.25</wheel_separation>
        <wheel_radius>0.3</wheel_radius>
        <odom_publish_frequency>1</odom_publish_frequency>
        <max_linear_acceleration>1</max_linear_acceleration>
        <min_linear_acceleration>-1</min_linear_acceleration>
        <max_angular_acceleration>2</max_angular_acceleration>
        <min_angular_acceleration>-2</min_angular_acceleration>
        <max_linear_velocity>0.5</max_linear_velocity>
        <min_linear_velocity>-0.5</min_linear_velocity>
        <max_angular_velocity>1</max_angular_velocity>
        <min_angular_velocity>-1</min_angular_velocity>
      </plugin>
</gazebo>
```