from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='imu_driver',  
            executable='talker',   
            name='imu_driver_node',
            parameters=[{
                'port': LaunchConfiguration('port', default='/dev/ttyUSB0'),
                'baudrate': LaunchConfiguration('baudrate', default='115200'),
                'sampling_rate': LaunchConfiguration('sampling_rate', default='40')
            }],
            output='screen'
        )
    ])