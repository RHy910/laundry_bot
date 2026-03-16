from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='laundry_dt',
            executable='ultrasonic_publisher',
            name='ultrasonic_publisher'
        ),
        Node(
            package='laundry_dt',
            executable='dc_motor_controller',
            name='dc_motor_controller'
        ),
        Node(
            package='laundry_dt',
            executable='dc_motor_subscriber',
            name='dc_motor_subscriber'
        ),
    ])