#!/usr/bin/env python3
# Copyright 2022-2024 Fraunhofer Italia Research

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

# Copyright 2022 Fraunhofer Italia Research. All Rights Reserved.

# Small programm for contacting the manager and spawning a plug-in

import argparse

import sys
import rospkg
import rclpy
import time
from rclpy.node import Node

from rosbim_manager_msgs.msg import Plugin
from rosbim_manager_msgs.srv import SpawnPlugin


class Spawner(Node):
    def __init__(self, plugin_name, config):
        super().__init__("rosbim_spawner")
        ## Main loop: Parses the input arguments, waits for rosbim_manager to come on and sends a request
        plugin = Plugin()

        plugin.package_name = plugin_name
        plugin.config_file = config
        spawn_plugin = SpawnPlugin.Request()
        spawn_plugin.plugin = plugin

        self.get_logger().info("Waiting for service server to connect...")
        self.cli = self.create_client(SpawnPlugin, "/rosbim_manager/spawn_plugin")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for server.")
            time.sleep(0.1)
        self.get_logger().info("Connected to service server.")

        req = SpawnPlugin.Request()
        req.plugin = plugin
        self.future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, self.future)

        if self.future.result().success:
            self.get_logger().info(
                "Plug-in '" + plugin.package_name + "' spawned successfully."
            )
        else:
            self.get_logger().error(
                "Plug-in '"
                + plugin.package_name
                + "' could not be spawned successfully!"
            )
        return None


def main(args=None):
    rclpy.init(args=args)
    format_class = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(
        formatter_class=format_class, description="Default info"
    )
    parser.add_argument(
        "--name",
        required=True,
        type=str,
        help="Name of the package containing the plug-in that should be spawned.",
        default=None,
    )
    parser.add_argument(
        "--config",
        type=str,
        help="Path of the configuration file that should be used for configuring the service names.",
        default="",
    )

    command_line_args = rclpy.utilities.remove_ros_args(args=sys.argv)[1:]
    known_args, unknown_args = parser.parse_known_args(command_line_args)
    plugin_name = known_args.name
    config = known_args.config

    node = Spawner(plugin_name, config)


if __name__ == "__main__":
    main()
