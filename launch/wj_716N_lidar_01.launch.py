from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    hostname = LaunchConfiguration('hostname', default='192.168.0.2')
    port = LaunchConfiguration('port', default='"2110"')
    frequency_scan = LaunchConfiguration('frequency_scan', default='1')
    min_ang = LaunchConfiguration('min_ang', default='-2.35619449')
    max_ang = LaunchConfiguration('max_ang', default='2.35619449')
    frame_id = LaunchConfiguration('frame_id', default='laser')
    range_min = LaunchConfiguration('range_min', default='0.0')
    range_max = LaunchConfiguration('range_max', default='30.0')

    wj_716N_lidar_node = Node(
        package='wj_716N_lidar',
        executable='wj_716N_lidar',
        name='wj_716N_lidar_01',
        output='screen',
        parameters=[
            {'hostname': hostname},
            {'port': port},
            {'frequency_scan': frequency_scan},
            {'min_ang': min_ang},
            {'max_ang': max_ang},
            {'frame_id': frame_id},
            {'range_min': range_min},
            {'range_max': range_max}
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            name = 'hostname',
            default_value= hostname,
            description='IP address of the scanner'
        ),
        DeclareLaunchArgument(
            name = 'port',
            default_value = port,
            description='Port number of the scanner'
        ),
        DeclareLaunchArgument(
            name = 'frequency_scan',
            default_value=frequency_scan,
            description='Scan frequency (1: 0.25°_15hz, 2: 0.25°_25hz)'
        ),
        DeclareLaunchArgument(
            name = 'min_ang',
            default_value=min_ang,
            description='Minimum scan angle in radians'
        ),
        DeclareLaunchArgument(
            name = 'max_ang',
            default_value=max_ang,
            description='Maximum scan angle in radians'
        ),
        DeclareLaunchArgument(
            name = 'frame_id',
            default_value=frame_id,
            description='Frame ID of the scanner'
        ),
        DeclareLaunchArgument(
            name = 'range_min',
            default_value=range_min,
            description='Minimum range value'
        ),
        DeclareLaunchArgument(
            name = 'range_max',
            default_value=range_max,
            description='Maximum range value'
        ),
        wj_716N_lidar_node
    ])
if __name__ == '__main__':
    generate_launch_description()