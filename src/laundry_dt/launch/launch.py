from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='laundry_dt',
            executable='staircase_publisher',
            name='staircase_publisher'
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
        Node(
            package='laundry_dt',
            executable='stepper_controller',
            name='stepper_controller'
        ),
        Node(
            package='laundry_dt',
            executable='stepper_subscriber',
            name='stepper_subscriber'
        ),
        Node(
            package='laundry_dt',
            executable='robot_controller',
            name='robot_controller'
        ),
    ])