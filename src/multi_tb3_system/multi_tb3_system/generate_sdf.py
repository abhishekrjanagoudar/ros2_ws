#!/usr/bin/env python3
"""
Utility module — NOT a ROS node.
"""

import os
import re
import logging
from ament_index_python.packages import get_package_share_directory

logger = logging.getLogger(__name__)

# Topic replacement map

_TOPIC_PATCHES = [
    # DiffDrive implicit topics

    # Explicit topic tags (sensor / JointStatePublisher)
    (r'<topic>scan</topic>',         '<topic>/{ns}/scan</topic>'),
    (r'<topic>imu</topic>',          '<topic>/{ns}/imu</topic>'),
    (r'<topic>joint_states</topic>', '<topic>/{ns}/joint_states</topic>'),

    # DiffDrive explicit tags (present in some SDF versions)
    (r'<topic>cmd_vel</topic>',      '<topic>/{ns}/cmd_vel</topic>'),
    (r'<odom_topic>odom</odom_topic>',
     '<odom_topic>/{ns}/odom</odom_topic>'),
    (r'<tf_topic>/tf</tf_topic>',    '<tf_topic>/tf</tf_topic>'),   # keep global

    # Frame IDs (published in odometry message header)
    (r'<frame_id>odom</frame_id>',
     '<frame_id>{ns}/odom</frame_id>'),
    (r'<child_frame_id>base_footprint</child_frame_id>',
     '<child_frame_id>{ns}/base_footprint</child_frame_id>'),
    (r'<gz_frame_id>base_scan</gz_frame_id>',
     '<gz_frame_id>{ns}/base_scan</gz_frame_id>'),
]


# DiffDrive topic injection

_DIFFDRIVE_PLUGIN_RE = re.compile(
    r'(<plugin[^>]*gz-sim-diff-drive-system[^>]*>)(.*?)(</plugin>)',
    re.DOTALL,
)

def _inject_diffdrive_topics(content: str, ns: str) -> str:
    """
Ensure the DiffDrive plugin block contains explicit <topic>, <odom_topic>,
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


# Public API

def generate_robot_sdf(ns: str, output_dir: str = '/tmp') -> str:
    """
Generate a namespaced TurtleBot3 Burger SDF for robot *ns*.
"""
    # Locate template
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

    # Patch model name so Gazebo entity name matches namespace
    content = re.sub(
        r'<model name="[^"]*">',
        f'<model name="{ns}">',
        content,
        count=1,
    )

    # Inject DiffDrive topics if missing
    content = _inject_diffdrive_topics(content, ns)

    # Apply all remaining topic / frame patches
    for pattern, replacement in _TOPIC_PATCHES:
        replacement_str = replacement.replace('{ns}', ns)
        content = re.sub(pattern, replacement_str, content)

    # Write output
    os.makedirs(output_dir, exist_ok=True)
    output_path = os.path.join(output_dir, f'multi_tb3_{ns}.sdf')
    with open(output_path, 'w', encoding='utf-8') as f:
        f.write(content)

    logger.info(f'[generate_sdf] Written: {output_path}')
    return output_path
