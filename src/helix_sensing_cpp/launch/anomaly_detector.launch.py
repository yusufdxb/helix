"""
Launch anomaly detector — C++ port gated behind use_cpp_anomaly arg.

Default is `use_cpp_anomaly:=false` — Python stays the default until
hardware parity is re-confirmed on the Jetson with a real rosbag. Set
the arg true to launch the C++ lifecycle component instead. Both paths
honor the same config file (same param names).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode, Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_cpp = LaunchConfiguration("use_cpp_anomaly")
    params_file = LaunchConfiguration("params_file")

    default_params = PathJoinSubstitution(
        [FindPackageShare("helix_sensing_cpp"), "config", "helix_params.yaml"]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_cpp_anomaly",
            default_value="false",
            description=(
                "If true, launch the C++ helix_sensing_cpp anomaly detector "
                "in place of the default Python helix_core node."
            ),
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params,
            description="YAML file with zscore_threshold / consecutive_trigger / "
                        "window_size / emit_cooldown_s params.",
        ),

        # C++ path: lifecycle node, auto-configure/activate is left to the
        # bringup layer (matches the Python node's behavior in helix_bringup).
        LifecycleNode(
            package="helix_sensing_cpp",
            executable="helix_anomaly_detector",
            name="helix_anomaly_detector",
            namespace="",
            parameters=[params_file],
            condition=IfCondition(use_cpp),
            output="screen",
        ),

        # Python default path — delegates to helix_core's console script.
        Node(
            package="helix_core",
            executable="helix_anomaly_detector",
            name="helix_anomaly_detector",
            parameters=[params_file],
            condition=UnlessCondition(use_cpp),
            output="screen",
        ),
    ])
