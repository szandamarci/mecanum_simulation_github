import os
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    package_name = 'mecanum_simulation_github'
    bringup_dir = get_package_share_directory('nav2_bringup')
    local_dir = get_package_share_directory(package_name)
    world_file = os.path.join(local_dir, 'worlds/empty.sdf')
    rviz_dir = os.path.join(local_dir, 'rviz')
    xacro_file = os.path.join(local_dir, 'urdf/mecanum.xacro')
    description_raw = xacro.process_file(xacro_file).toxml()
    joy_config = launch.substitutions.LaunchConfiguration('joy_config')
    joy_dev = launch.substitutions.LaunchConfiguration('joy_dev')
    publish_stamped_twist = launch.substitutions.LaunchConfiguration('publish_stamped_twist')
    config_filepath = launch.substitutions.LaunchConfiguration('config_filepath')

    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    slam = LaunchConfiguration('slam')
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')

    gazebo_models_path, ignore_last_dir = os.path.split(local_dir)
    os.environ["GZ_SIM_RESOURCE_PATH"] += os.pathsep + gazebo_models_path

    rviz = Node(package= 'rviz2',
                namespace='',
                executable='rviz2',
                name='rviz2',
                arguments=[
                    '-d', [os.path.join(rviz_dir, 'mecanum.rviz')]],
                output='screen')
    
    start_nav_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(local_dir, 'launch/nav2.launch.py')),
        launch_arguments={
            'map' : map_file,
            'use_sim_time' : 'True',
            'params_file' : params_file,
            'slam' : slam 
        }.items())
    
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map', 
        default_value=os.path.join(
            bringup_dir, 'maps', 'turtlebot3_world.yaml'),
        description='Path to map file'
        )
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file', 
        default_value=os.path.join(
            local_dir, 'params', 'nav2_params_four.yaml'),
        description='Path to nav2 parameters file'
        )
    
    declare_slam_cmd = DeclareLaunchArgument(
        'slam', 
        default_value='True',
        description='Run slam or not'
        )
     
    joy = Node(package ='joy',
               executable='joy_node',
               name='joy_node',
               parameters=[
                   {
                       'device_id': joy_dev,
                       'deadzone': 0.3,
                       'autorepeat_rate': 20.0,
                   }
               ]
    )

    teleop = Node(package= 'teleop_twist_joy',
                namespace='',
                executable='teleop_node',
                name='teleop_twist_joy_node',
                parameters=[config_filepath, {'publish_stamped_twist': publish_stamped_twist}],
                remappings={('/cmd_vel', launch.substitutions.LaunchConfiguration('joy_vel'))}
                )

    declare_joy_vel = DeclareLaunchArgument('joy_vel', default_value='cmd_vel')
    declare_joy_config = DeclareLaunchArgument('joy_config', default_value='xbox')
    declare_joy_dev = DeclareLaunchArgument('joy_dev', default_value='0')
    declare_publish_stamped_twist = DeclareLaunchArgument('publish_stamped_twist', default_value='false')
    declare_config_filepath = DeclareLaunchArgument('config_filepath', default_value=[
        launch.substitutions.TextSubstitution(text=os.path.join(
            get_package_share_directory('teleop_twist_joy'), 'config', '')),
            joy_config, launch.substitutions.TextSubstitution(text='.config.yaml')])
    
    world_arg = DeclareLaunchArgument(
        'world', default_value=world_file
    )

    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(local_dir, 'launch', 'world.launch.py')),
            launch_arguments={'world': LaunchConfiguration('world')}.items())

    spawn_xacro = Node(package= "ros_gz_sim",
                        executable="create",
                        arguments=[
                            "-name", "my_robot",
                            "-topic", "robot_description",
                            "-x", "0.0", "-y", "0.0", "-z", "0.5", "-Y", "0.0"],
                        output='screen',
                        parameters=[
                            {'use_sim_time': True},
                        ])

    robot_state_publisher_node = Node(
                        package= 'robot_state_publisher',
                        executable='robot_state_publisher',
                        parameters=[
                            {'robot_description' : description_raw,
                             'use_sim_time' : use_sim_time}],
                        output='screen',
                        remappings=[
                                ('/tf', 'tf'),
                                ('/tf_static', 'tf_static')

                        ])
    
    gz_bridge_node = Node(
                        package="ros_gz_bridge",
                        executable="parameter_bridge",
                        arguments=[
                            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                            "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
                            "/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry",
                            "/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model",
                            "/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V",
                            "/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan",
                        ],
                        output="screen",
                        parameters=[
                            {'use_sim_time': True},
                        ])
    
    return LaunchDescription([
        declare_map_yaml_cmd,
        declare_params_file_cmd,
        declare_slam_cmd,
        declare_joy_vel,
        declare_joy_config,
        declare_joy_dev,
        declare_publish_stamped_twist,
        declare_config_filepath,

        world_arg,
        world_launch,
        spawn_xacro,
        robot_state_publisher_node,
        gz_bridge_node,

        rviz,
        start_nav_cmd,
        joy, 
        teleop

    ])


