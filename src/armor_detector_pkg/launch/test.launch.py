import launch
import launch_ros
def generate_launch_description():
    action_declare_arg_mode = launch.actions.DeclareLaunchArgument(
        'launch_arg_mode', 
        default_value='0'
    )
    action_node_rviz2_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        output='screen'
    )
    action_node_image_processer_node = launch_ros.actions.Node(
        package = 'armor_detector_pkg',
        executable ='image_processer_node',
        output = 'screen',
        parameters= [{'mode':launch.substitutions.LaunchConfiguration('launc_arg_mode', default='0')}]
    )
    action_node_number_recognizer_node = launch_ros.actions.Node(
        package='armor_detector_pkg',
        executable='number_recognizer_node',
        output='screen'
    )
    action_node_video_player_node = launch_ros.actions.Node(
        package='armor_detector_pkg',
        executable='video_player_node',
        output='screen'
    )
    action_node_aim_predictor_node = launch_ros.actions.Node(
        package='armor_detector_pkg',
        executable='aim_predictor_node',
        output='screen'
    )
    action_node_video_capturer_node = launch_ros.actions.Node(
        package='armor_detector_pkg',
        executable='video_capturer_node',
        output='screen'
    )
    return launch.LaunchDescription([
        action_declare_arg_mode,
        action_node_image_processer_node,
        #action_node_number_recognizer_node,
        action_node_video_player_node,
        action_node_aim_predictor_node,
        action_node_video_capturer_node
    ])