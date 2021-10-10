# Copyright 2021 Christophe Bedard
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

import launch
import launch_ros.actions
from tracetools_launch.action import Trace


def generate_launch_description():
    # Arguments
    count_arg = launch.actions.DeclareLaunchArgument(
        'count',
        default_value='5',
        description='number of pings and pongs in a sequence',
    )
    period_arg = launch.actions.DeclareLaunchArgument(
        'period',
        default_value='3',
        description='period for timer that triggers new ping/pong sequences',
    )
    sequences_arg = launch.actions.DeclareLaunchArgument(
        'sequences',
        default_value='5',
        description='number of ping/pong sequences to do',
    )
    # Trace
    trace = Trace(
        session_name='ros-world-2021-demo',
        events_kernel=[],  # Disable kernel tracing
    )
    # Nodes
    ping = launch_ros.actions.Node(
        package='ros_world_2021_demo',
        executable='ping',
        arguments=[
            'count', launch.substitutions.LaunchConfiguration(count_arg.name),
            'period', launch.substitutions.LaunchConfiguration(period_arg.name),
            'sequences', launch.substitutions.LaunchConfiguration(sequences_arg.name),
        ],
        output='screen',
    )
    pong = launch_ros.actions.Node(
        package='ros_world_2021_demo',
        executable='pong',
        output='screen',
    )

    return launch.LaunchDescription([
        count_arg,
        period_arg,
        sequences_arg,
        trace,
        ping,
        pong,
        # Shut down when ping node exits
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=ping,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        ),
    ])
