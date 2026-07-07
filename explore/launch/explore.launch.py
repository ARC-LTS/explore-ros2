import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    config = os.path.join(
        get_package_share_directory('explore_lite'), 'config', 'params.yaml'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace = LaunchConfiguration('namespace')

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    parameters = [config, {'use_sim_time': use_sim_time}]
    # radiation_scale/radiation_gradient_scale/dose_budget_scale are only ever
    # read once, at construction (this->get_parameter(...) into a cached
    # member) -- there is no on_set_parameters callback, so a runtime
    # `ros2 param set` on a running node has zero effect on behavior even
    # though it updates the parameter server's stored value. This must be
    # set here, at launch time, to actually realize a radiation-agnostic
    # baseline arm.
    if LaunchConfiguration('radiation_agnostic').perform(context) == 'true':
        parameters.append({
            'radiation_scale': 0.0,
            'radiation_gradient_scale': 0.0,
            'dose_budget_scale': 0.0,
        })

    node = Node(
        package='explore_lite',
        name='explore_node',
        namespace=namespace,
        executable='explore',
        parameters=parameters,
        output='screen',
        remappings=remappings,
    )
    return [node]


def generate_launch_description():
    ld = LaunchDescription()

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use simulation/Gazebo clock'
    )
    declare_namespace_argument = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace for the explore node',
    )
    declare_radiation_agnostic_argument = DeclareLaunchArgument(
        'radiation_agnostic',
        default_value='false',
        description='Zero out radiation-informed frontier cost weights at construction '
                    'time, realizing a radiation-agnostic baseline arm. Must be set at '
                    'launch time (see comment at the parameters list above).',
    )

    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_namespace_argument)
    ld.add_action(declare_radiation_agnostic_argument)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
