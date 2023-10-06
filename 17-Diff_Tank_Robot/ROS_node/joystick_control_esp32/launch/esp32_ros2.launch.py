from launch import LaunchDescription
from launch_ros.actions import Node
 
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2'
        ),

        Node(
            package='joystick_control_esp32',
            executable='joystick_node',
            name='joystick_node'
        ),

        Node(
            package='joy',
            executable='joy_node',
            name='joy_node'
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ["0.15", "0", "0", "0", "0", "0", "base_link", "ultrasonar"],
            name='ultrasonar_tf'
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ["0.1", "-0.03", "0.1", "0", "0", "0", "base_link", "mpu6050"],
            name='mpu6050_tf'
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ["0.12", "0", "0.1", "0", "0", "0", "base_link", "gps"],
            name='gps_tf'
        ),

    ])