import os
import launch
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.actions import (DeclareLaunchArgument, EmitEvent, RegisterEventHandler)
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch_ros.actions import LifecycleNode, Node #Agregar este import
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition

def generate_launch_description():

    # パラメータファイルのパス設定
    config_file_path = os.path.join(
        get_package_share_directory('urg_node2'),
        'config',
        'params_serial.yaml'
    )

    # パラメータファイルのロード
    with open(config_file_path, 'r') as file:
        config_params = yaml.safe_load(file)['urg_node2']['ros__parameters']

    # urg_node2をライフサイクルノードとして起動
    lifecycle_node = LifecycleNode(
        package='urg_node2',
        executable='urg_node2_node',
        name=LaunchConfiguration('node_name'),
        remappings=[('scan', LaunchConfiguration('scan_topic_name'))],
        parameters=[config_params],
        namespace='',
        output='screen',
    )

    # Unconfigure状態からInactive状態への遷移（auto_startがtrueのとき実施）
    urg_node2_node_configure_event_handler = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=lifecycle_node,
            on_start=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(lifecycle_node),
                        transition_id=Transition.TRANSITION_CONFIGURE,
                   ),
                ),
            ],
        ),
        condition=IfCondition(LaunchConfiguration('auto_start')),
    )

    # Inactive状態からActive状態への遷移（auto_startがtrueのとき実施）
    urg_node2_node_activate_event_handler = RegisterEventHandler(
        event_handler=OnStateTransition(
            target_lifecycle_node=lifecycle_node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(lifecycle_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    ),
                ),
            ],
        ),
       condition=IfCondition(LaunchConfiguration('auto_start')),
    )

    # Nodo para publicar la transformada estática del LIDAR
    static_transform_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.15', '0', '0.2', '0', '0', '0', 'base_link', 'laser'],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('auto_start', default_value='true'),
        DeclareLaunchArgument('node_name', default_value='urg_node2'),
        DeclareLaunchArgument('scan_topic_name', default_value='scan'),
        lifecycle_node,
        urg_node2_node_configure_event_handler,
        urg_node2_node_activate_event_handler,
        static_transform_publisher,  # Añade este nodo
    ])