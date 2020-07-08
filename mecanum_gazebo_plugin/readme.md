# Macanum Gazebo Plugin

This plugin simulates a mecanum wheel using a passive roller joint and a sphere collision shape.
For this to work, the wheel URDF has to be modified as follows:

 - The wheel joint should be `continuous` and acutated, e.g. using a transmission with an approriate hardware interface and the ros_control plugin.
 - The wheel link should contain the visual elelment and a dummy mass and inertia but no collision shape.
 - A second `continuous` joint should be added with zero origin. The axis is not important but as a convention it should be the same as the wheel joint axis. The joint should have a proxy roller link as a child.
 - The proxy roller link should contain the wheel mass and inertia. It should also contain a sphere collision shape whe the same radius as the wheel.

 The proxy roller link emulates the passive rolling of the wheel. This plugin will then align the axis of the passive joint so that it matches with the axis of the roller that would be in contact with ground. 
 
 The plugin has two parameters:
  - `wheelLinkName`: the name of the wheel link
  - `axis`: axis of the passive roller joint in the wheel frame, e.g., for mecanum wheels at 45deg angle this would be `[1 1 0]` (or similar, depending on how the wheel is mounted). The axis will be normalized.

Friction parameters can be tuned via gazebo in the URDF. The wheel control can be set up and tuned to match the robot using ros_control and its gazebo plugin. The sphere collision model is not an accurate representation of the mecanum wheel. It allows the wheel to drive over obstacles sideways and the curvature of the sphere may not match the curvature of the rollers for all widths. It does allow the wheel to drive on uneven terrain and model the friction and it produces motion very similar to the real world behaviour.

This plugin can also be used to simulate omni wheels by specifying the axis to be perpendicular to wheel axis.

### Example

Additional link and joint in the URDF:
```
<!-- Proxy roller link -->
<link name="wheel_link_fl_passive">
    <inertial>
        <mass value="10.0"/>
        <origin rpy="0 0 0" xyz="0 0 0"/>
        <inertia ixx="0.0441" ixy="0" ixz="0" iyy="0.0441" iyz="0" izz="0.0441"/>
    </inertial>
    <visual>
        <origin rpy="0 0 0" xyz="0 0 0"/>
        <geometry>
        <sphere radius="0.001"/>
        </geometry>
        <transparency>1</transparency>
    </visual>
    <collision>
        <origin rpy="0 0 0" xyz="0 0 0"/>
        <geometry>
        <sphere radius="0.105"/>
        </geometry>
    </collision>
</link>

<!-- Passive joint -->
<joint name="wheel_joint_fl_passive" type="continuous">
    <axis xyz="0 1 0"/>
    <limit effort="100" velocity="100"/>
    <dynamics damping="0.0" friction="0.0"/>
    <safety_controller k_velocity="10.0"/>
    <origin rpy="0 0 0" xyz="0 0 0"/>
    <parent link="wheel_link_fl"/>
    <child link="wheel_link_fl_passive"/>
</joint>

<!-- Actuation of the wheel -->
<transmission name="wheel_joint_fl_trans">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="wheel_joint_fl">
    <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>
    </joint>
    <actuator name="wheel_joint_fl_motor">
    <mechanicalReduction>1</mechanicalReduction>
    </actuator>
</transmission>
```

Plugin configuration for 4 mecanum wheels:
```
<plugin name="wheel_controller_fl" filename="libmecanum_gazebo_plugin.so">
    <wheelLinkName>wheel_link_fl</wheelLinkName>
    <axis>-1 1 0</axis>
</plugin>
<plugin name="wheel_controller_fr" filename="libmecanum_gazebo_plugin.so">
    <wheelLinkName>wheel_link_fr</wheelLinkName>
    <axis>-1 -1 0</axis>
</plugin>
<plugin name="wheel_controller_rl" filename="libmecanum_gazebo_plugin.so">
    <wheelLinkName>wheel_link_rl</wheelLinkName>
    <axis>1 1 0</axis>
</plugin>
<plugin name="wheel_controller_rr" filename="libmecanum_gazebo_plugin.so">
    <wheelLinkName>wheel_link_rr</wheelLinkName>
    <axis>1 -1 0</axis>
</plugin>
```
