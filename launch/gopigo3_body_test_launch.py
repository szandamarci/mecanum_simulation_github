import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro
 
 
def generate_launch_description():
 
    # Specify the name of the package and path to xacro file within the package

    pkg_name = 'gopigo_custom'

    file_subpath = 'urdf/gopigo3.xacro'
 
    # Use xacro to process the file
    xacro_file = os.path.join(get_package_share_directory(pkg_name),file_subpath)
    description_raw = xacro.process_file(xacro_file).toxml()
 
 
    # Configure the node

    robot_state_publisher = Node(

        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': description_raw,

        'use_sim_time': False}] # add other parameters here if required

    )
 
    joint_state_publisher = Node(

        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen',
        parameters=[{'robot_description': description_raw,

        'use_sim_time': False}] # add other parameters here if required

    )
 
    
 
    # Run the node

    return LaunchDescription([

        robot_state_publisher,
        joint_state_publisher

    ])