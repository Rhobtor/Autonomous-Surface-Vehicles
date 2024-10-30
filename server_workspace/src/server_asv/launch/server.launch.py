from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node,SetParameter
from launch.actions import TimerAction, DeclareLaunchArgument,ExecuteProcess, OpaqueFunction, GroupAction, IncludeLaunchDescription, RegisterEventHandler, LogInfo
from ament_index_python.packages import get_package_share_directory
import os
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition

def launch_setup(context, *args, **kwargs):
    #as launch argument whether to create a bag
    debug = LaunchConfiguration('debug')
    debug_bool=context.perform_substitution(debug).lower() in "true"
    debug_arg = SetParameter(name="debug", value=debug)

    default_config_common = os.path.join( #path of common yaml file for camera node but with gnss enabled
        get_package_share_directory('server_asv'),'config','config.yaml')

    algorithm_node= LaunchDescription([Node(
            package='server_asv', executable='algorithm',
            remappings=[],
            parameters=[default_config_common],
            name='algorithm_node'
            )
    ])

    ship_communications_node= LaunchDescription([Node(
            package='server_asv', executable='ship_communications',
            remappings=[],
            parameters=[default_config_common],
            name='ship_communications_node'
            )
    ])

    state_composer_node= LaunchDescription([Node(
            package='server_asv', executable='state_composer',
            remappings=[],
            parameters=[default_config_common],
            name='state_composer_node'
            )
    ])

    logger_node= LaunchDescription([Node(
            package='server_asv', executable='logger',
            remappings=[],
            parameters=[default_config_common],
            name='logger_node'
            )
    ])
            
    return [
        debug_arg,
        algorithm_node,
        ship_communications_node,
        state_composer_node,
        logger_node,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('debug',  default_value='False'),
        OpaqueFunction(function=launch_setup),
    ])
