"""Launch the modular GICP localizer without owning vehicle bring-up or TF."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _optional_bool(context, name):
    value = LaunchConfiguration(name).perform(context).strip().lower()
    if not value:
        return None
    if value not in ("true", "false"):
        raise RuntimeError(f"{name} must be true or false")
    return value == "true"


def _launch_localizer(context):
    package_share = get_package_share_directory("gicp_interface")
    default_config = os.path.join(
        package_share, "param", "gicp_interface.param.yaml"
    )
    params = [default_config]

    config_path = LaunchConfiguration("config_path").perform(context).strip()
    if config_path:
        config_path = os.path.realpath(config_path)
        if not os.path.isfile(config_path):
            raise RuntimeError(f"GICP config not found: {config_path}")
        params.append(config_path)

    config_overlay_path = LaunchConfiguration("config_overlay_path").perform(
        context
    ).strip()
    if config_overlay_path:
        config_overlay_path = os.path.realpath(config_overlay_path)
        if not os.path.isfile(config_overlay_path):
            raise RuntimeError(
                f"GICP config overlay not found: {config_overlay_path}"
            )
        params.append(config_overlay_path)

    concat_requested = _optional_bool(context, "lidar_concat_enabled")
    concat_source = LaunchConfiguration("lidar_concat_source").perform(
        context
    ).strip().lower()
    if concat_source not in ("iris_interface", "gicp_internal_paused"):
        raise RuntimeError(
            "lidar_concat_source must be iris_interface or "
            "gicp_internal_paused"
        )

    require_all_aux = _optional_bool(context, "require_all_aux")
    pointcloud_topic = LaunchConfiguration("pointcloud_topic").perform(
        context
    ).strip()
    lidar_frame = LaunchConfiguration("lidar_frame").perform(context).strip()

    # lidar_concat_enabled selects the input product. iris_interface is the
    # active provider; the old in-node merge remains available only through an
    # explicitly named paused fallback.
    node_concat_override = None
    if concat_requested is True and concat_source == "iris_interface":
        node_concat_override = False
        pointcloud_topic = pointcloud_topic or "/lidar/points"
        lidar_frame = lidar_frame or "base_link"
        if require_all_aux is True:
            raise RuntimeError(
                "require_all_aux belongs to the paused GICP-internal concat; "
                "iris_interface may publish a partial bucket after its timeout"
            )
        input_contract_log = (
            "LiDAR input: iris_interface art-jazzy external concat, "
            f"topic={pointcloud_topic}, frame={lidar_frame}, "
            "fixed_qos=SensorDataQoS/BEST_EFFORT; "
            "GICP internal concat is disabled."
        )
    elif concat_requested is True:
        node_concat_override = True
        pointcloud_topic = pointcloud_topic or "/luminar_front/points"
        input_contract_log = (
            "PAUSED CAPABILITY selected explicitly: GICP internal concat, "
            f"primary_topic={pointcloud_topic}, frame={lidar_frame or 'profile'}."
        )
    else:
        if concat_requested is False:
            node_concat_override = False
            mode = "single-LiDAR"
        else:
            mode = "profile/default"
        if concat_requested is False and require_all_aux is True:
            raise RuntimeError(
                "require_all_aux requires lidar_concat_enabled:=true"
            )
        pointcloud_topic = pointcloud_topic or "/luminar_front/points"
        input_contract_log = (
            f"LiDAR input: {mode}, topic={pointcloud_topic}, "
            f"frame={lidar_frame or 'profile'}."
        )

    overrides = {
        "use_sim_time": LaunchConfiguration("use_sim_time"),
    }
    if lidar_frame:
        overrides["localization/lidar_frame"] = lidar_frame
    if node_concat_override is not None:
        overrides["localization/lidar_concat/enabled"] = node_concat_override

    # Tuning launch arguments are optional overrides. Leaving them empty must
    # preserve config_path values; otherwise a clean launch silently replaces
    # a validated profile with generic launch defaults.
    optional_int_overrides = {
        "scan_max_points": "scan_max_points",
        "gicp_max_iterations": "gicp/maxIterations",
        "executor_threads": "localization/executor_threads",
        "primary_queue_size": "localization/lidar_concat/primary_queue_size",
        "gicp_num_threads": "gicp/numThreads",
        "gicp_max_inner_iterations": "gicp/maxInnerIterations",
        "gicp_startup_accepted_scans": "gicp/startupAcceptedScans",
        "gicp_startup_max_iterations": "gicp/startupMaxIterations",
    }
    optional_float_overrides = {
        "scan_downsample": "scan_downsample",
        "gicp_finalization_reserve_ms": "gicp/finalizationReserveMs",
        "future_aux_wait_timeout_s": (
            "localization/lidar_concat/future_aux_wait_timeout_s"
        ),
        "map_voxel_size": "localization/map_voxel_size",
        "local_map_radius_m": "localization/local_map/radius_m",
        "local_map_rebuild_distance_m": (
            "localization/local_map/rebuild_distance_m"
        ),
        "registration_rate_hz": "localization/registration_rate_hz",
        "gicp_max_optimization_time_ms": "gicp/maxOptimizationTimeMsCooperative",
        "gicp_startup_max_optimization_time_ms": (
            "gicp/startupMaxOptimizationTimeMsCooperative"
        ),
    }
    optional_bool_overrides = {
        "imu_only": "localization/imu_only",
        "require_all_aux": "localization/lidar_concat/require_all_aux",
        "local_map_enabled": "localization/local_map/enabled",
        "debug_verbose_scan_log": "localization/debug/verbose_scan_log",
        "track_time_tuning_enabled": "localization/track_tuning/enable",
    }
    for launch_name, parameter_name in optional_int_overrides.items():
        value = LaunchConfiguration(launch_name).perform(context).strip()
        if value:
            overrides[parameter_name] = int(value)
    for launch_name, parameter_name in optional_float_overrides.items():
        value = LaunchConfiguration(launch_name).perform(context).strip()
        if value:
            overrides[parameter_name] = float(value)
    for launch_name, parameter_name in optional_bool_overrides.items():
        value = _optional_bool(context, launch_name)
        if value is not None:
            overrides[parameter_name] = value

    track_timing_topic = LaunchConfiguration("track_timing_topic").perform(
        context
    ).strip()
    if track_timing_topic:
        overrides["localization/track_tuning/rde_telemetry_topic"] = (
            track_timing_topic
        )

    map_path = LaunchConfiguration("map_path").perform(context).strip()
    if not map_path:
        raise RuntimeError(
            "map_path is required; pass map_path:=/absolute/path/to/map.pcd"
        )
    map_path = os.path.realpath(map_path)
    if not os.path.isfile(map_path):
        raise RuntimeError(f"GICP map not found: {map_path}")
    overrides["localization/map_path"] = map_path

    urdf_path = LaunchConfiguration("urdf_path").perform(context).strip()
    if urdf_path:
        overrides["localization/lidar_concat/urdf_path"] = os.path.realpath(
            urdf_path
        )

    ttl_directory = LaunchConfiguration("ttl_directory").perform(context).strip()
    if ttl_directory:
        ttl_directory = os.path.realpath(ttl_directory)
        if not os.path.isdir(ttl_directory):
            raise RuntimeError(f"TTL directory not found: {ttl_directory}")
        overrides["localization/track_constraint/ttl_directory"] = ttl_directory

    params.append(overrides)
    return [
        LogInfo(msg=input_contract_log),
        Node(
            package="gicp_interface",
            executable="gicp_interface_node",
            name="gicp_interface",
            output="screen",
            parameters=params,
            remappings=[
                ("pointcloud", pointcloud_topic),
                ("imu", LaunchConfiguration("imu_topic")),
                ("gt_odom", LaunchConfiguration("gt_odom_topic")),
            ],
        )
    ]


def generate_launch_description():
    package_share = get_package_share_directory("gicp_interface")
    default_vehicle_config = os.path.join(
        package_share, "param", "laguna.param.yaml"
    )
    arguments = [
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument(
            "map_path",
            default_value="",
            description="Required absolute or relative path to the map PCD.",
        ),
        DeclareLaunchArgument(
            "config_path", default_value=default_vehicle_config
        ),
        DeclareLaunchArgument("config_overlay_path", default_value=""),
        DeclareLaunchArgument(
            "pointcloud_topic",
            default_value="",
            description=(
                "Optional override. Defaults to /luminar_front/points for "
                "single/internal input and /lidar/points for iris concat."
            ),
        ),
        DeclareLaunchArgument("imu_topic", default_value="/vks/imu"),
        DeclareLaunchArgument(
            "gt_odom_topic", default_value="/vks/filtered_odom"
        ),
        DeclareLaunchArgument(
            "lidar_frame",
            default_value="",
            description=(
                "Optional input-frame contract override. Defaults to "
                "the YAML profile, or base_link for iris_interface concat."
            ),
        ),
        DeclareLaunchArgument("imu_only", default_value=""),
        # Empty means "preserve the selected YAML profile". These remain
        # explicit launch overrides for front-only diagnostics and replay QoS.
        DeclareLaunchArgument(
            "lidar_concat_enabled",
            default_value="",
            description=(
                "true selects a concat product; iris_interface is the default "
                "provider. false selects the primary LiDAR only."
            ),
        ),
        DeclareLaunchArgument(
            "lidar_concat_source",
            default_value="iris_interface",
            description=(
                "Concat provider: iris_interface (active default) or "
                "gicp_internal_paused (retained paused fallback)."
            ),
        ),
        DeclareLaunchArgument(
            "require_all_aux",
            default_value="",
            description=(
                "Strict merge gate for gicp_internal_paused only; not "
                "available for iris_interface output."
            ),
        ),
        DeclareLaunchArgument("executor_threads", default_value=""),
        DeclareLaunchArgument("future_aux_wait_timeout_s", default_value=""),
        DeclareLaunchArgument("primary_queue_size", default_value=""),
        DeclareLaunchArgument("map_voxel_size", default_value=""),
        DeclareLaunchArgument("local_map_enabled", default_value=""),
        DeclareLaunchArgument("local_map_radius_m", default_value=""),
        DeclareLaunchArgument(
            "local_map_rebuild_distance_m", default_value=""
        ),
        DeclareLaunchArgument("registration_rate_hz", default_value=""),
        DeclareLaunchArgument("gicp_num_threads", default_value=""),
        DeclareLaunchArgument("gicp_max_inner_iterations", default_value=""),
        DeclareLaunchArgument(
            "gicp_max_optimization_time_ms", default_value=""
        ),
        DeclareLaunchArgument(
            "gicp_finalization_reserve_ms", default_value=""
        ),
        DeclareLaunchArgument(
            "gicp_startup_max_optimization_time_ms", default_value=""
        ),
        DeclareLaunchArgument("gicp_startup_accepted_scans", default_value=""),
        DeclareLaunchArgument("gicp_startup_max_iterations", default_value=""),
        DeclareLaunchArgument("scan_downsample", default_value=""),
        DeclareLaunchArgument("scan_max_points", default_value=""),
        DeclareLaunchArgument("gicp_max_iterations", default_value=""),
        DeclareLaunchArgument("debug_verbose_scan_log", default_value=""),
        DeclareLaunchArgument("track_time_tuning_enabled", default_value=""),
        DeclareLaunchArgument("track_timing_topic", default_value=""),
        DeclareLaunchArgument("urdf_path", default_value=""),
        DeclareLaunchArgument("ttl_directory", default_value=""),
    ]
    return LaunchDescription(arguments + [OpaqueFunction(function=_launch_localizer)])
