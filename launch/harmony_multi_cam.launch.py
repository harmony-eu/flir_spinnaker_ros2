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

from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument as LaunchArg
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import ComposableNodeContainer, PushRosNamespace
from launch_ros.descriptions import ComposableNode

from ament_index_python.packages import get_package_share_directory

# Common camera parameters
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
    """Launch multiple camera nodes."""
    flir_dir = get_package_share_directory('flir_spinnaker_ros2')
    config_dir = flir_dir + '/config/'
    path_info = 'package://flir_spinnaker_ros2/camera_info'

    ld = LaunchDescription()
    
    # Common launch args
    ld.add_action(LaunchArg('ns', default_value='flir', description='namespace'))
    ld.add_action(LaunchArg('frame_rate', default_value='20', description='frame rate'))
    ld.add_action(LaunchArg('rectified', default_value='true', description='whether to launch a rectification node for this camera'))

    # Camera-specific launch args
    ld.add_action(LaunchArg('cam_0_name', default_value='left', description='camera 0 name'))
    ld.add_action(LaunchArg('cam_1_name', default_value='right', description='camera 1 name'))
    ld.add_action(LaunchArg('cam_2_name', default_value='back', description='camera 2 name'))
    ld.add_action(LaunchArg('serial_0', default_value="'22112356'", description='camera 0 serial'))
    ld.add_action(LaunchArg('serial_1', default_value="'22115684'", description='camera 1 serial'))
    ld.add_action(LaunchArg('serial_2', default_value="'22112349'", description='camera 2 serial'))
    ld.add_action(LaunchArg('cam_0_info', default_value='22112356.yaml', description='camera 0 info YAML file'))
    ld.add_action(LaunchArg('cam_1_info', default_value='22115684.yaml', description='camera 1 info YAML file'))
    ld.add_action(LaunchArg('cam_2_info', default_value='22112349.yaml', description='camera 2 info YAML file'))
    ld.add_action(LaunchArg('frame_id_0', default_value='flir_left_link', description='camera 0 frame id in the TF tree'))
    ld.add_action(LaunchArg('frame_id_1', default_value='flir_right_link', description='camera 1 frame id in the TF tree'))
    ld.add_action(LaunchArg('frame_id_2', default_value='flir_back_link', description='camera 2 frame id in the TF tree'))
    
    # List of (composable) camera nodes
    cam_nodes = [
        ComposableNode(
            package='flir_spinnaker_ros2',
            plugin='flir_spinnaker_ros2::CameraDriver',
            name=LaunchConfiguration('cam_0_name'),
            namespace=LaunchConfiguration('ns'),
            parameters=[camera_params,
                        {'parameter_file': config_dir + 'firefly.cfg',
                         'serial_number': [LaunchConfiguration('serial_0')],
                         'camerainfo_url': PathJoinSubstitution([path_info, 
                                                                 LaunchConfiguration('cam_0_info')
                                                                ]),
                         'frame_id': LaunchConfiguration('frame_id_0'),
                         'frame_rate': LaunchConfiguration('frame_rate')
                        }
                        ]
        ),
        ComposableNode(
            package='flir_spinnaker_ros2',
            plugin='flir_spinnaker_ros2::CameraDriver',
            name=LaunchConfiguration('cam_1_name'),
            namespace=LaunchConfiguration('ns'),
            parameters=[camera_params,
                        {'parameter_file': config_dir + 'firefly.cfg',
                         'serial_number': [LaunchConfiguration('serial_1')],
                         'camerainfo_url': PathJoinSubstitution([path_info, 
                                                                 LaunchConfiguration('cam_1_info')
                                                                ]),
                         'frame_id': LaunchConfiguration('frame_id_1'),
                         'frame_rate': LaunchConfiguration('frame_rate')
                        }
                        ]
        ),
        ComposableNode(
            package='flir_spinnaker_ros2',
            plugin='flir_spinnaker_ros2::CameraDriver',
            name=LaunchConfiguration('cam_2_name'),
            namespace=LaunchConfiguration('ns'),
            parameters=[camera_params,
                        {'parameter_file': config_dir + 'firefly.cfg',
                         'serial_number': [LaunchConfiguration('serial_2')],
                         'camerainfo_url': PathJoinSubstitution([path_info, 
                                                                 LaunchConfiguration('cam_2_info')
                                                                ]),
                         'frame_id': LaunchConfiguration('frame_id_2'),
                         'frame_rate': LaunchConfiguration('frame_rate')
                        }
                        ]
        ),
    ]

    # List of (composable) rectify nodes
    rect_nodes = [
        # Rectify image
        ComposableNode(
            package='image_proc',
            plugin='image_proc::RectifyNode',
            name='rectify_color_node',
            namespace=[LaunchConfiguration('ns'), "/", LaunchConfiguration('cam_0_name')], 
            remappings=[
                # Subscriber remap
                ('image', 'image_raw'),
                ('camera_info', 'camera_info'),
                # Publisher remap
                ('image_rect', 'image_rect')
            ],
        ),
        ComposableNode(
            package='image_proc',
            plugin='image_proc::RectifyNode',
            name='rectify_color_node',
            namespace=[LaunchConfiguration('ns'), "/", LaunchConfiguration('cam_2_name')], 
            remappings=[
                # Subscriber remap
                ('image', 'image_raw'),
                ('camera_info', 'camera_info'),
                # Publisher remap
                ('image_rect', 'image_rect')
            ],
        ),
        ComposableNode(
            package='image_proc',
            plugin='image_proc::RectifyNode',
            name='rectify_color_node',
            namespace=[LaunchConfiguration('ns'), "/", LaunchConfiguration('cam_1_name')], 
            remappings=[
                # Subscriber remap
                ('image', 'image_raw'),
                ('camera_info', 'camera_info'),
                # Publisher remap
                ('image_rect', 'image_rect')
            ],
        ),
    ]
    

    # Composable container with only camera nodes (if rectified = false)
    ld.add_action(ComposableNodeContainer(
                    name='camera_container',
                    namespace=LaunchConfiguration('ns'),
                    package='rclcpp_components',
                    executable='component_container',
                    output='screen',
                    condition=IfCondition(LaunchConfiguration('rectified')),
                    composable_node_descriptions=cam_nodes+rect_nodes
            )
    )
    
    # Composable container with only camera nodes (if rectified = true)
    ld.add_action(ComposableNodeContainer(
                    name='camera_container',
                    namespace=LaunchConfiguration('ns'),
                    package='rclcpp_components',
                    executable='component_container',
                    output='screen',
                    condition=UnlessCondition(LaunchConfiguration('rectified')),
                    composable_node_descriptions=cam_nodes
            )
    )

    return ld