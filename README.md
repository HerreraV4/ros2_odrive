# ros_odrive in ROS 2

## Compiling the package

This package has been modified for ROS 2. The option for torque control has been incorporated.

Before compiling this package, the folder named "ros_odrive_msgs" must be moved to the path ~/$NAME_WORKSPACE/src and compiled with colcon:

```
cd ~/$NAME_WORKSPACE
colcon build --packages-select ros_odrive_msgs
```

After compiling the package containing the messages, compile the ros2_odrive package with:

```
cd ~/$NAME_WORKSPACE
colcon build --packages-select ros2_odrive
```

## Executing the package

To run the node, use the command line below:

```
ros2 run ros2_odrive odrive --ros-args -p od_sn:='"0xserial_number"'
```

Where serial_number is the number of the odrive board shown in odrivetool.

### Example for sending a value in torque control

To publish a message in the control topic of the board, use the command line below:

```
ros2 topic pub /odrive_ctrl_0xserial_number ros2_odrive_msgs/msg/Odrvctrl "{target: 0, command: 6, axis: 0, fval: 0.0, fval2: 0.0}"
```
Where command: 6 is the option for torque control, and fval: 0.0 is the desired torque value.

