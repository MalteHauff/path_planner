#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _as_bool(s: str) -> bool:
    return str(s).strip().lower() in ("1", "true", "yes", "on")

def _as_int(s: str) -> int:
    return int(float(s))  # tolerate "30.0"

def _as_float(s: str) -> float:
    return float(s)

def _maybe_set(target: dict, key: str, value_str: str, caster):
    if value_str is not None and value_str != "":
        target[key] = caster(value_str)


def launch_setup(context, *args, **kwargs):
    params_from_cli = {}

    # Time / frames / TF
    _maybe_set(params_from_cli, "use_sim_time",               LaunchConfiguration("use_sim_time").perform(context),               _as_bool)
    _maybe_set(params_from_cli, "world_frame",                LaunchConfiguration("world_frame").perform(context),                _as_float if False else str)  # keep as string
    _maybe_set(params_from_cli, "base_frame",                 LaunchConfiguration("base_frame").perform(context),                 _as_float if False else str)  # keep as string
    _maybe_set(params_from_cli, "tf_lookup_timeout_s",        LaunchConfiguration("tf_lookup_timeout_s").perform(context),        _as_float)
    _maybe_set(params_from_cli, "tf_poll_hz",                 LaunchConfiguration("tf_poll_hz").perform(context),                 _as_float)

    # Core planner knobs
    _maybe_set(params_from_cli, "heading_resolution_deg",     LaunchConfiguration("heading_resolution_deg").perform(context),     _as_float)
    _maybe_set(params_from_cli, "vehicle_width_m",            LaunchConfiguration("vehicle_width_m").perform(context),            _as_float)
    _maybe_set(params_from_cli, "vehicle_length_m",           LaunchConfiguration("vehicle_length_m").perform(context),           _as_float)
    _maybe_set(params_from_cli, "inflate_cells",              LaunchConfiguration("inflate_cells").perform(context),              _as_int)

    # Replan hysteresis
    _maybe_set(params_from_cli, "min_replan_translation",     LaunchConfiguration("min_replan_translation").perform(context),     _as_float)
    _maybe_set(params_from_cli, "min_replan_rotation_deg",    LaunchConfiguration("min_replan_rotation_deg").perform(context),    _as_float)

    # Search / safety
    _maybe_set(params_from_cli, "max_iterations",             LaunchConfiguration("max_iterations").perform(context),             _as_int)
    _maybe_set(params_from_cli, "planning_clearance_m",       LaunchConfiguration("planning_clearance_m").perform(context),       _as_float)
    _maybe_set(params_from_cli, "occ_threshold",              LaunchConfiguration("occ_threshold").perform(context),              _as_int)

    # Final publish policy
    _maybe_set(params_from_cli, "final_allow_unknown",        LaunchConfiguration("final_allow_unknown").perform(context),        _as_bool)
    _maybe_set(params_from_cli, "publish_truncated_path",     LaunchConfiguration("publish_truncated_path").perform(context),     _as_bool)
    _maybe_set(params_from_cli, "min_publish_length_m",       LaunchConfiguration("min_publish_length_m").perform(context),       _as_float)
    _maybe_set(params_from_cli, "min_publish_points",         LaunchConfiguration("min_publish_points").perform(context),         _as_int)
    _maybe_set(params_from_cli, "min_publish_ratio",          LaunchConfiguration("min_publish_ratio").perform(context),          _as_float)

    # Anti-flap (stickiness)
    _maybe_set(params_from_cli, "keep_old_if_valid_prefix",   LaunchConfiguration("keep_old_if_valid_prefix").perform(context),   _as_bool)
    _maybe_set(params_from_cli, "lock_prefix_m",              LaunchConfiguration("lock_prefix_m").perform(context),              _as_float)
    _maybe_set(params_from_cli, "max_prefix_deviation_m",     LaunchConfiguration("max_prefix_deviation_m").perform(context),     _as_float)
    _maybe_set(params_from_cli, "require_improvement_m",      LaunchConfiguration("require_improvement_m").perform(context),      _as_float)

    # Goal hysteresis
    _maybe_set(params_from_cli, "goal_pos_tol_m",             LaunchConfiguration("goal_pos_tol_m").perform(context),             _as_float)
    _maybe_set(params_from_cli, "goal_yaw_tol_deg",           LaunchConfiguration("goal_yaw_tol_deg").perform(context),           _as_float)
    _maybe_set(params_from_cli, "stop_replanning_within_goal_radius_m",
               LaunchConfiguration("stop_replanning_within_goal_radius_m").perform(context),                                      _as_float)
    _maybe_set(params_from_cli, "post_goal_freeze_s",         LaunchConfiguration("post_goal_freeze_s").perform(context),         _as_float)
    _maybe_set(params_from_cli, "publish_when_goal_reached",  LaunchConfiguration("publish_when_goal_reached").perform(context),  _as_bool)

    # Path warmup/keepalive
    _maybe_set(params_from_cli, "warmup_republish_s",         LaunchConfiguration("warmup_republish_s").perform(context),         _as_float)
    _maybe_set(params_from_cli, "keepalive_rate_hz",          LaunchConfiguration("keepalive_rate_hz").perform(context),          _as_float)
    _maybe_set(params_from_cli, "enable_keepalive",           LaunchConfiguration("enable_keepalive").perform(context),           _as_bool)

    params_file = LaunchConfiguration("params_file").perform(context)

    # Topic remaps
    map_topic  = LaunchConfiguration("map_topic").perform(context)
    goal_topic = LaunchConfiguration("goal_topic").perform(context)
    path_topic = LaunchConfiguration("path_topic").perform(context)

    node = Node(
        package="path_planner",
        executable="graph_search_node",
        name="graph_search_node",
        output="screen",
        parameters=[
            params_file,        # base YAML (typed)
            params_from_cli     # typed overrides from CLI
        ],
        remappings=[
            ("map", map_topic),                     # FROM relative "map"
            ("goal_pose", goal_topic),              # FROM relative "goal_pose"
            ("external_path_from_ros1", path_topic) # FROM relative "external_path_from_ros1"
        ]

    )

    return [node]


def generate_launch_description():
    declare_args = [
        # YAML with defaults
        DeclareLaunchArgument(
            "params_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("path_planner"), "config", "graph_search_node.yaml"
            ]),
            description="Full path to a YAML file with node parameters."
        ),

        # Time / frames / TF
        DeclareLaunchArgument("use_sim_time", default_value="", description="Use /clock (true in sim)."),
        DeclareLaunchArgument("world_frame",  default_value="", description="Global frame (e.g., map)."),
        DeclareLaunchArgument("base_frame",   default_value="", description="Robot base frame (e.g., base_link)."),
        DeclareLaunchArgument("tf_lookup_timeout_s", default_value="", description="TF lookup timeout (s)."),
        DeclareLaunchArgument("tf_poll_hz",   default_value="", description="TF polling rate for start pose (Hz)."),

        # Core planner knobs
        DeclareLaunchArgument("heading_resolution_deg", default_value="", description="Heading discretization (deg)."),
        DeclareLaunchArgument("vehicle_width_m",  default_value="", description="Robot width (m)."),
        DeclareLaunchArgument("vehicle_length_m", default_value="", description="Robot length (m)."),
        DeclareLaunchArgument("inflate_cells",    default_value="", description="Static inflation radius (cells)."),

        # Replan hysteresis
        DeclareLaunchArgument("min_replan_translation",  default_value="", description="Meters moved to trigger replan."),
        DeclareLaunchArgument("min_replan_rotation_deg", default_value="", description="Yaw change (deg) to trigger replan."),

        # Search / safety
        DeclareLaunchArgument("max_iterations",       default_value="", description="Hybrid A* iteration cap."),
        DeclareLaunchArgument("planning_clearance_m", default_value="", description="Extra margin (m) for planning only."),
        DeclareLaunchArgument("occ_threshold",        default_value="", description="Occupancy threshold for obstacles."),

        # Final publish policy
        DeclareLaunchArgument("final_allow_unknown",    default_value="", description="Final check: unknown=free if true."),
        DeclareLaunchArgument("publish_truncated_path", default_value="", description="Publish safe prefix if tail blocked."),
        DeclareLaunchArgument("min_publish_length_m",   default_value="", description="Min truncated length (m)."),
        DeclareLaunchArgument("min_publish_points",     default_value="", description="Min truncated points."),
        DeclareLaunchArgument("min_publish_ratio",      default_value="", description="Min fraction that must pass final check."),

        # Anti-flap (stickiness)
        DeclareLaunchArgument("keep_old_if_valid_prefix", default_value="", description="Keep old path if prefix valid."),
        DeclareLaunchArgument("lock_prefix_m",            default_value="", description="Meters of prefix to compare."),
        DeclareLaunchArgument("max_prefix_deviation_m",   default_value="", description="Max RMS deviation to still switch."),
        DeclareLaunchArgument("require_improvement_m",    default_value="", description="Min remaining-length improvement."),

        # Goal hysteresis
        DeclareLaunchArgument("goal_pos_tol_m",                    default_value="", description="Goal position tolerance (m)."),
        DeclareLaunchArgument("goal_yaw_tol_deg",                  default_value="", description="Goal yaw tolerance (deg)."),
        DeclareLaunchArgument("stop_replanning_within_goal_radius_m", default_value="", description="No replan near goal radius (m)."),
        DeclareLaunchArgument("post_goal_freeze_s",                default_value="", description="Freeze replans after goal (s)."),
        DeclareLaunchArgument("publish_when_goal_reached",         default_value="", description="Keep publishing during freeze."),

        # Path warmup/keepalive
        DeclareLaunchArgument("warmup_republish_s", default_value="", description="Burst window after accepting a path (s)."),
        DeclareLaunchArgument("keepalive_rate_hz",  default_value="", description="Keepalive publish rate (Hz)."),
        DeclareLaunchArgument("enable_keepalive",   default_value="", description="Enable periodic keepalive republish."),

        # Topic remaps
        DeclareLaunchArgument("map_topic",  default_value="/map",                       description="OccupancyGrid topic."),
        DeclareLaunchArgument("goal_topic", default_value="/goal_pose",                 description="Goal pose topic."),
        DeclareLaunchArgument("path_topic", default_value="/external_path_from_ros1",   description="Published nav_msgs/Path topic."),
    ]

    return LaunchDescription(declare_args + [OpaqueFunction(function=launch_setup)])
