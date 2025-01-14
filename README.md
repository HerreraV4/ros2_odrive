# ros_odrive in ROS 2

This package has been modified for ROS 2. The option for torque control has been incorporated.

Before to compile this package, the folder named "ros_odrive_msgs" must move it in path ~/$NAME_WORKSPACE/src and compile with colcon:

'''bash
cd ~/$NAME_WORKSPACE
colcon build --packages-select ros_odrive_msgs
'''

After compile the package that contain the messages, compile the package ros2_odrive with:

'''bash
cd ~/$NAME_WORKSPACE
colcon build --packages-select ros2_odrive
'''
