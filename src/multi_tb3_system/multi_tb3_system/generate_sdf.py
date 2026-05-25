#!/usr/bin/env python3
"""
generate_sdf.py
===============
Utility module — NOT a ROS node.

Generates a per-robot namespaced SDF file from the package-local
turtlebot3_burger model.sdf template by patching all Gazebo plugin
topic strings to absolute, robot-specific paths.

Why this is needed
------------------
The DiffDrive and JointStatePublisher plugins in model.sdf use *bare*
(relative) topic names such as ``cmd_vel``, ``odom``, and ``joint_states``.
Gazebo Sim (Harmonic / gz-sim 8) resolves relative sensor topics under
``/model/<name>/…`` but plugin topics (DiffDrive, JointStatePublisher) are
published as-is at the transport layer — i.e. the bare string becomes a
global Gazebo topic.  This causes ALL robots to share ``/cmd_vel``,
``/odom``, and ``/joint_states``, making multi-robot impossible.

Fix: replace every plugin topic with an absolute path like ``/tb1/cmd_vel``
so each robot gets its own isolated Gazebo transport channel that the
ros_gz_bridge can bridge 1-to-1 to the matching ROS namespace.

Usage (from a launch file via OpaqueFunction)::

    from multi_tb3_system.generate_sdf import generate_robot_sdf

    sdf_path = generate_robot_sdf('tb2')
    # → writes /tmp/multi_tb3_tb2.sdf and returns the path

The returned path is passed directly to ``ros_gz_sim create -file <path>``.
"""

import os
import re
import logging
from ament_index_python.packages import get_package_share_directory

logger = logging.getLogger(__name__)

# ─── Topic replacement map ─────────────────────────────────────────────────────
# Each tuple: (pattern_to_match, replacement_template)
# {ns} is substituted with the robot namespace at call time.
#
# Covers:
#   gz-sim-diff-drive-system     : cmd_vel topic, odom_topic, tf_topic
#   gz-sim-joint-state-publisher : joint_states topic
#   gpu_lidar sensor             : scan topic
#   imu sensor                   : imu topic
#
# All replacements use *absolute* Gz topic paths (leading /) so that
# Gazebo Sim does NOT scope them further under /model/<name>/.

_TOPIC_PATCHES = [
    # ── DiffDrive implicit topics ─────────────────────────────────────────────
    # DiffDrive defaults: cmd_vel → subscribe, odom → publish, /tf → publish.
    # They are not always explicit in the SDF; we inject <topic>, <odom_topic>
    # and <tf_topic> tags INSIDE the DiffDrive plugin block if missing.
    # See: _inject_diffdrive_topics()

    # ── Explicit topic tags (sensor / JointStatePublisher) ────────────────────
    (r'<topic>scan</topic>',         '<topic>/{ns}/scan</topic>'),
    (r'<topic>imu</topic>',          '<topic>/{ns}/imu</topic>'),
    (r'<topic>joint_states</topic>', '<topic>/{ns}/joint_states</topic>'),

    # ── DiffDrive explicit tags (present in some SDF versions) ───────────────
    (r'<topic>cmd_vel</topic>',      '<topic>/{ns}/cmd_vel</topic>'),
    (r'<odom_topic>odom</odom_topic>',
     '<odom_topic>/{ns}/odom</odom_topic>'),
    (r'<tf_topic>/tf</tf_topic>',    '<tf_topic>/tf</tf_topic>'),   # keep global

    # ── Frame IDs (published in odometry message header) ─────────────────────
    (r'<frame_id>odom</frame_id>',
     '<frame_id>{ns}/odom</frame_id>'),
    (r'<child_frame_id>base_footprint</child_frame_id>',
     '<child_frame_id>{ns}/base_footprint</child_frame_id>'),
    (r'<gz_frame_id>base_scan</gz_frame_id>',
     '<gz_frame_id>{ns}/base_scan</gz_frame_id>'),
]


# ─── DiffDrive topic injection ─────────────────────────────────────────────────

_DIFFDRIVE_PLUGIN_RE = re.compile(
    r'(<plugin[^>]*gz-sim-diff-drive-system[^>]*>)(.*?)(</plugin>)',
    re.DOTALL,
)

def _inject_diffdrive_topics(content: str, ns: str) -> str:
    """
    Ensure the DiffDrive plugin block contains explicit <topic>, <odom_topic>,
    and <tf_topic> tags.  If they are already present the regex patches below
    will handle them; if absent we inject them before </plugin>.

    This handles both old and new SDF styles.
    """
    def _patch_plugin(match):
        open_tag  = match.group(1)
        body      = match.group(2)
        close_tag = match.group(3)

        injections = []
        if '<topic>' not in body:
            injections.append(f'      <topic>/{ns}/cmd_vel</topic>')
        if '<odom_topic>' not in body:
            injections.append(f'      <odom_topic>/{ns}/odom</odom_topic>')
        if '<tf_topic>' not in body:
            injections.append('      <tf_topic>/tf</tf_topic>')

        if injections:
            body = body.rstrip() + '\n' + '\n'.join(injections) + '\n    '
        return open_tag + body + close_tag

    return _DIFFDRIVE_PLUGIN_RE.sub(_patch_plugin, content)


# ─── Public API ───────────────────────────────────────────────────────────────

def generate_robot_sdf(ns: str, output_dir: str = '/tmp') -> str:
    """
    Generate a namespaced TurtleBot3 Burger SDF for robot *ns*.

    Reads the SDF template from the package's ``models/`` directory,
    applies all topic/frame patches, and writes the result to
    ``<output_dir>/multi_tb3_<ns>.sdf``.

    Args:
        ns:         Robot namespace, e.g. ``'tb1'``, ``'tb2'``, ``'tb3'``.
        output_dir: Directory to write the generated SDF (default: /tmp).

    Returns:
        Absolute path to the generated SDF file.

    Raises:
        FileNotFoundError: If the template SDF is not found.
        IOError:           If the output file cannot be written.
    """
    # ── Locate template ───────────────────────────────────────────────────────
    pkg_share = get_package_share_directory('multi_tb3_system')
    template_path = os.path.join(
        pkg_share, 'models', 'turtlebot3_burger', 'model.sdf'
    )
    if not os.path.isfile(template_path):
        raise FileNotFoundError(
            f'[generate_sdf] Template SDF not found: {template_path}'
        )

    with open(template_path, 'r', encoding='utf-8') as f:
        content = f.read()

    # ── Patch model name so Gazebo entity name matches namespace ──────────────
    content = re.sub(
        r'<model name="[^"]*">',
        f'<model name="{ns}">',
        content,
        count=1,
    )

    # ── Inject DiffDrive topics if missing ────────────────────────────────────
    content = _inject_diffdrive_topics(content, ns)

    # ── Apply all remaining topic / frame patches ─────────────────────────────
    for pattern, replacement in _TOPIC_PATCHES:
        replacement_str = replacement.replace('{ns}', ns)
        content = re.sub(pattern, replacement_str, content)

    # ── Write output ──────────────────────────────────────────────────────────
    os.makedirs(output_dir, exist_ok=True)
    output_path = os.path.join(output_dir, f'multi_tb3_{ns}.sdf')
    with open(output_path, 'w', encoding='utf-8') as f:
        f.write(content)

    logger.info(f'[generate_sdf] Written: {output_path}')
    return output_path
