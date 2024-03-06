import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    hostname = LaunchConfiguration('hostname', default='192.168.0.2')
    port = LaunchConfiguration('port', default='2110')
    frequency_scan = LaunchConfiguration('frequency_scan', default='1')
    min_ang = LaunchConfiguration('min_ang', default='-2.35619449')
    max_ang = LaunchConfiguration('max_ang', default='2.35619449')
    frame_id = LaunchConfiguration('frame_id', default='laser')
    range_min = LaunchConfiguration('range_min', default='0')
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
            'hostname',
            default_value='192.168.0.2',
            description='IP address of the scanner'
        ),
        DeclareLaunchArgument(
            'port',
            default_value='2110',
            description='Port number of the scanner'
        ),
        DeclareLaunchArgument(
            'frequency_scan',
            default_value='1',
            description='Scan frequency (1: 0.25°_15hz, 2: 0.25°_25hz)'
        ),
        DeclareLaunchArgument(
            'min_ang',
            default_value='-2.35619449',
            description='Minimum scan angle in radians'
        ),
        DeclareLaunchArgument(
            'max_ang',
            default_value='2.35619449',
            description='Maximum scan angle in radians'
        ),
        DeclareLaunchArgument(
            'frame_id',
            default_value='laser',
            description='Frame ID of the scanner'
        ),
        DeclareLaunchArgument(
            'range_min',
            default_value='0',
            description='Minimum range value'
        ),
        DeclareLaunchArgument(
            'range_max',
            default_value='30.0',
            description='Maximum range value'
        ),
        wj_716N_lidar_node
    ])

if __name__ == '__main__':
    launch.main()

# <?xml version="1.1"?>
# <!-- start and stop angle is given in [rad] -->
# <!--
# default min_angle is -135 degree.
# default max_angle is +135 degree.  

# Check IP-address, if you scanner is not found after roslaunch.
# -->

# <launch>
#   <node name="wj_716N_lidar_01" pkg="wj_716N_lidar" type="wj_716N_lidar" respawn="false" output="screen">
#   <param name="hostname"         type="string"  value="192.168.0.2" />
#   <param name="port"             type="string"  value="2110" />
#   <param name="frequency_scan"   type="int"     value="1" />    <!-- 1: 0.25°_15hz,
#                                                                      2: 0.25°_25hz,-->
#   <!-- -135° -->
#   <param name="min_ang"          type="double"  value="-2.35619449" />
#   <!-- 135° -->
#   <param name="max_ang"          type="double"  value="2.35619449" />
#   <param name="frame_id"         type="str"     value="laser" />
#   <param name="range_min"        type="double"  value="0" />
#   <param name="range_max"        type="double"  value="30.0" />

#   </node>
# </launch>

# <!--
# Conversion between degree and rad

# DEG	RAD
# -180	-3.141592654
# -175	-3.054326191
# -170	-2.967059728
# -165	-2.879793266
# -160	-2.792526803
# -155	-2.705260341
# -150	-2.617993878
# -145	-2.530727415
# -140	-2.443460953
# -137.5	-2,3998277
# -135	-2.35619449
# -130	-2.268928028
# -125	-2.181661565
# -120	-2.094395102
# -115	-2.00712864
# -110	-1.919862177
# -105	-1.832595715
# -100	-1.745329252
# -95	-1.658062789
# -90	-1.570796327
# -85	-1.483529864
# -80	-1.396263402
# -75	-1.308996939
# -70	-1.221730476
# -65	-1.134464014
# -60	-1.047197551
# -55	-0.959931089
# -50	-0.872664626
# -45	-0.785398163
# -40	-0.698131701
# -35	-0.610865238
# -30	-0.523598776
# -25	-0.436332313
# -20	-0.34906585
# -15	-0.261799388
# -10	-0.174532925
# -5	-0.087266463
# 0	0
# 5	0.087266463
# 10	0.174532925
# 15	0.261799388
# 20	0.34906585
# 25	0.436332313
# 30	0.523598776
# 35	0.610865238
# 40	0.698131701
# 45	0.785398163
# 50	0.872664626
# 55	0.959931089
# 60	1.047197551
# 65	1.134464014
# 70	1.221730476
# 75	1.308996939
# 80	1.396263402
# 85	1.483529864
# 90	1.570796327
# 95	1.658062789
# 100	1.745329252
# 105	1.832595715
# 110	1.919862177
# 115	2.00712864
# 120	2.094395102
# 125	2.181661565
# 130	2.268928028
# 135	2.35619449
# 137.5	2,3998277
# 140	2.443460953
# 145	2.530727415
# 150	2.617993878
# 155	2.705260341
# 160	2.792526803
# 165	2.879793266
# 170	2.967059728
# 175	3.054326191
# 180	3.141592654
# 185	3.228859116
# 190	3.316125579
# 195	3.403392041
# 200	3.490658504
# 205	3.577924967
# 210	3.665191429
# 215	3.752457892
# 220	3.839724354
# 225	3.926990817
# 230	4.01425728
# 235	4.101523742
# 240	4.188790205
# 245	4.276056667
# 250	4.36332313
# 255	4.450589593
# 260	4.537856055
# 265	4.625122518
# 270	4.71238898
# 275	4.799655443
# 280	4.886921906
# 285	4.974188368
# 290	5.061454831
# 295	5.148721293
# 300	5.235987756
# 305	5.323254219
# 310	5.410520681
# 315	5.497787144
# 320	5.585053606
# 325	5.672320069
# 330	5.759586532
# 335	5.846852994
# 340	5.934119457
# 345	6.021385919
# 350	6.108652382
# 355	6.195918845
# 360	6.283185307
# -->
