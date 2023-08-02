# -----------------------------------------------------------------------------
# Copyright 2023 Fabio Amadio <fabioamadio93@gmail.com>
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
#

from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument as LaunchArg
from launch.actions import GroupAction
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, PushRosNamespace
from launch_ros.descriptions import ComposableNode
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory

camera_params = {
    'debug': False,
    'compute_brightness': False,
    'adjust_timestamp': True,
    'dump_node_map': False,
    # set parameters defined in firefly.cfg
    'gain_auto': 'Continuous',
    'pixel_format': 'RGB8Packed',
    'isp_enable': True,
    'exposure_auto': 'Continuous',
    'balance_white_auto': 'Continuous',
    # 'device_link_throughput_limit': 380000000,
    # ---- to reduce the sensor width and shift the crop
    # 'image_width': 1408,
    # 'image_height': 1080,
    # 'offset_x': 16,
    # 'offset_y': 0,
    'frame_rate_auto': 'Off',
    'frame_rate_enable': True,
    # 'buffer_queue_size': 1,
    'trigger_mode': 'Off',
    'chunk_mode_active': True,
    'chunk_selector_frame_id': 'FrameID',
    'chunk_enable_frame_id': True,
    'chunk_selector_exposure_time': 'ExposureTime',
    'chunk_enable_exposure_time': True,
    'chunk_selector_gain': 'Gain',
    'chunk_enable_gain': True,
    'chunk_selector_timestamp': 'Timestamp',
    'chunk_enable_timestamp': True,
    }


def generate_launch_description():
    """Launch camera node."""
    flir_dir = get_package_share_directory('flir_spinnaker_ros2')
    config_dir = flir_dir + '/config/'

    ns_arg = LaunchArg('ns', default_value='flir', description='namespace')
    name_arg = LaunchArg('camera_name', default_value='left', description='camera name')
    serial_arg = LaunchArg('serial', default_value="'22112356'", description='serial number')
    camera_info_arg = LaunchArg('camera_info', default_value='22112356.yaml', description='camera info YAML file')
    frame_rate_arg = LaunchArg('frame_rate', default_value='20', description='frame rate')
    frame_id_arg = LaunchArg('frame_id', default_value='flir_left_link', description='camera frame id in the TF tree e.g. flir_back_link')
    rectified_arg = LaunchArg('rectified', default_value='true', description='whether to launch a rectification node for this camera (default is false)')


    camera = Node(package='flir_spinnaker_ros2',
                executable='camera_driver_node',
                output='screen',
                name=LaunchConfiguration('camera_name'),
                namespace=LaunchConfiguration('ns'),
                parameters=[camera_params,
                            {'parameter_file': config_dir + 'firefly.cfg',
                             'serial_number': [LaunchConfiguration('serial')],
                             'camerainfo_url': PathJoinSubstitution(['package://flir_spinnaker_ros2/camera_info', LaunchConfiguration('camera_info')]),
                             'frame_id': LaunchConfiguration('frame_id'),
                             'frame_rate': LaunchConfiguration('frame_rate')
                            }
                           ],
                #remappings=[('~/control', '/exposure_control/control'), ('image', 'image_raw')]
                )

    composable_nodes = [
            # Rectify image
            ComposableNode(
                package='image_proc',
                plugin='image_proc::RectifyNode',
                name='rectify_color_node',
                namespace=LaunchConfiguration('camera_name'), 
                remappings=[
                    # Subscriber remap
                    ('image', 'image_raw'),
                    ('camera_info', 'camera_info'),
                    # Publisher remap
                    ('image_rect', 'image_rect')
                ],
            )]

    # Load composable container
    # Node loaded if 'rectified' is true
    image_processing = GroupAction(actions=[
                            PushRosNamespace(LaunchConfiguration('ns')),
                            ComposableNodeContainer(
                                name='image_proc_container',
                                namespace=LaunchConfiguration('camera_name'),
                                package='rclcpp_components',
                                executable='component_container',
                                output='screen',
                                condition=IfCondition(LaunchConfiguration('rectified')),
                                composable_node_descriptions=composable_nodes
                            )]
                       )

    return LaunchDescription([ns_arg,
                              name_arg,
                              serial_arg,
                              camera_info_arg, 
                              frame_rate_arg, 
                              frame_id_arg, 
                              rectified_arg, 
                              camera,
                              image_processing
                              ])
