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
    # Same launch-time float-cast pattern as rad_local_recovery_launch.py /
    # rad_safety_launch.py: explore_node's acute-dose yield threshold must
    # relax in lockstep with the recovery nodes' triggers (see
    # Explore::effectiveAcuteDoseThreshold for why a mismatch deadlocks).
    parameters.append({
        'dose_budget_relaxation_k':
            float(LaunchConfiguration('dose_relaxation_k').perform(context)),
        # Base round-trip transit-dose spend cap for the frontier target gate
        # (multiplied by the shared budget multiplier at run time); the
        # stage-5 sweep varies it per batch.
        'frontier_dose_spend_fraction':
            float(LaunchConfiguration('frontier_spend_fraction').perform(context)),
        # Information-gain-style attraction bonus toward elevated-but-unconfirmed
        # regions (see frontierCost's seek_bonus); 0.0 (default) reproduces the
        # prior avoidance-only frontier costing exactly.
        'radiation_seek_scale':
            float(LaunchConfiguration('radiation_seek_scale').perform(context)),
        # When true, radiation_seek_scale is ramped by progress deficit
        # (near-zero while exploration is healthy, full strength only once
        # the robot is approaching a goal-progress stall) instead of applied
        # as a flat constant. false (default) reproduces the constant-scale
        # behavior exactly.
        'radiation_seek_adaptive':
            LaunchConfiguration('radiation_seek_adaptive').perform(context) == 'true',
        # Boundary-survey ray count; fewer rays means less mission time spent
        # characterizing a source's extent at the cost of a coarser boundary
        # estimate. 8 (default) reproduces the original behavior.
        'radiation_survey_dirs':
            int(LaunchConfiguration('radiation_survey_dirs').perform(context)),
        # Time-budget alternative to radiation_survey_dirs: caps total
        # boundary-survey wall-clock time directly instead of reducing
        # angular coverage. 0.0 (default) disables the cap.
        'radiation_survey_time_cap_s':
            float(LaunchConfiguration('radiation_survey_time_cap_s').perform(context)),
    })
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
            # explore_node's own internal acute-dose yield (radiationCallback)
            # blacklists the current goal and stops to yield to
            # local_recovery_node -- a navigational reaction to radiation in
            # its own right, distinct from rad_safety's hard emergency brake
            # (which stays active in both arms per the test plan's design).
            # Neutralized here too so Baseline's frontier selection is never
            # influenced by radiation readings at all.
            'acute_dose_rate_mrem_hr': 1000000.0,
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
    declare_dose_relaxation_k_argument = DeclareLaunchArgument(
        'dose_relaxation_k',
        default_value='0.0',
        description='Dose-budget-aware relaxation coefficient for the acute-dose '
                    'yield threshold; must match the k passed to the recovery nodes.',
    )
    declare_frontier_spend_fraction_argument = DeclareLaunchArgument(
        'frontier_spend_fraction',
        default_value='0.05',
        description='Base fraction of the remaining dose budget a single frontier '
                    'round trip may spend (frontier target gate transit cap).',
    )
    declare_radiation_agnostic_argument = DeclareLaunchArgument(
        'radiation_agnostic',
        default_value='false',
        description='Zero out radiation-informed frontier cost weights at construction '
                    'time, realizing a radiation-agnostic baseline arm. Must be set at '
                    'launch time (see comment at the parameters list above).',
    )
    declare_radiation_seek_scale_argument = DeclareLaunchArgument(
        'radiation_seek_scale',
        default_value='0.0',
        description='Scale of the information-gain-style attraction bonus toward '
                    'elevated-but-unconfirmed regions in frontier costing. 0.0 '
                    '(default) reproduces the prior avoidance-only behavior exactly.',
    )
    declare_radiation_seek_adaptive_argument = DeclareLaunchArgument(
        'radiation_seek_adaptive',
        default_value='false',
        description='Ramp radiation_seek_scale by progress deficit instead of '
                    'applying it as a flat constant. false (default) reproduces '
                    'the constant-scale behavior exactly.',
    )
    declare_radiation_survey_dirs_argument = DeclareLaunchArgument(
        'radiation_survey_dirs',
        default_value='8',
        description='Boundary-survey ray count for RADIATION-mode source '
                    'characterization. 8 (default) reproduces the original behavior.',
    )
    declare_radiation_survey_time_cap_argument = DeclareLaunchArgument(
        'radiation_survey_time_cap_s',
        default_value='0.0',
        description='Cap total boundary-survey wall-clock time directly, '
                    'regardless of ray count visited. 0.0 (default) disables '
                    'the cap and reproduces the original behavior exactly.',
    )

    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_namespace_argument)
    ld.add_action(declare_dose_relaxation_k_argument)
    ld.add_action(declare_frontier_spend_fraction_argument)
    ld.add_action(declare_radiation_agnostic_argument)
    ld.add_action(declare_radiation_seek_scale_argument)
    ld.add_action(declare_radiation_seek_adaptive_argument)
    ld.add_action(declare_radiation_survey_dirs_argument)
    ld.add_action(declare_radiation_survey_time_cap_argument)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
