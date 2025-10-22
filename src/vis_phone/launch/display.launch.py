from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os
from ament_index_python.packages import get_package_share_path, get_package_share_directory


def generate_launch_description():

    show_rviz_arg = DeclareLaunchArgument(
        'show_rviz',
        default_value='true',   # must be string, not bool!
        description='Whether to show RViz'
    )

    show_rviz = LaunchConfiguration('show_rviz')

    urdf_path = os.path.join(get_package_share_path('vis_phone'),
                             'urdf', 'phone.urdf.xacro')

    rviz_config_path = os.path.join(get_package_share_path('vis_phone'),
                                    'rviz', 'my_phone.rviz')
    
    phone_orientation_params = os.path.join(
        get_package_share_directory("vis_phone"), 
        "config", "phone_orient_params.yaml")

    robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)

    phone_orientation_node = Node(
        package="phone_orientation",
        executable="phonesensor_node",
        parameters=[phone_orientation_params],
        remappings=[
           ('/tf','/phone/tf'),
        ]
    )

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=['-d', rviz_config_path],
        remappings=[
            ('/tf','/phone/tf'),
            ('/tf_static','/phone/tf_static'),
            ('/joint_state','/phone/joint_state'),
        ],
        output="screen",
        condition=IfCondition(show_rviz)
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace="phone",
        parameters=[{'robot_description': Command(['xacro ', urdf_path])}],
        remappings=[
            ('/tf','/phone/tf'),
            ('/tf_static','/phone/tf_static'),
            ('/joint_state','/phone/joint_state'),
        ],
        condition=IfCondition(show_rviz)
    )

    ld = LaunchDescription([
        show_rviz_arg,
        robot_state_publisher_node,
        rviz2_node,
        phone_orientation_node
    ])

    return ld
