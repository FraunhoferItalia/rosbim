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

import os
import yaml

import rospkg
import rclpy
from std_msgs.msg import String
from std_srvs.srv import Trigger

from rosbim_manager_msgs.msg import PluginState
from bim_interface.plugin_base import PluginBase
from ament_index_python.packages import get_package_share_directory


class DemoPlugin(PluginBase):
    ## Plug-in class for demonstration and testing purposes

    def _log(self, msg: str, publisher: rclpy.publisher.Publisher = None) -> None:
        ## Helper function for logging to console and topic
        #
        #  @param msg
        #    Message to be published
        #  @param publisher
        #    Optional publisher where the message should be published
        str_msg = String()
        str_msg.data = msg
        if publisher is not None:
            publisher.publish(str_msg)
        self.node.get_logger().info(msg)
        return None

    def _init(
        self,
        config_file_path: str = os.path.join(
            get_package_share_directory("rosbim_demo_plugin"), "config", "topics.yaml"
        ),
    ) -> bool:
        ## Initialise the topics for testing as well as class members
        #
        #  @param config_file_path
        #    The path of the package configuration file
        #  @return
        #    True if the plug-in was initialized successfully, else false is returned

        self.node.get_logger().info(config_file_path)
        self._service_name = None
        self._name = None
        self._srv = None
        self._init_pub = None
        self._start_pub = None
        self._stop_pub = None
        self._srv_pub = None

        f = open(config_file_path, "r")
        config_file = yaml.safe_load(f)

        if "service" in config_file:
            self._service_name = config_file["service"].get("name", None)

        if "testing" in config_file:
            init_topic_name = config_file["testing"].get("init_topic", None)
            start_topic_name = config_file["testing"].get("start_topic", None)
            stop_topic_name = config_file["testing"].get("stop_topic", None)
            service_topic_name = config_file["testing"].get("service_topic", None)

        # For testing purposes
        if init_topic_name is not None:
            self._init_pub = self.node.create_publisher(String, init_topic_name, 1)
        if start_topic_name is not None:
            self._start_pub = self.node.create_publisher(String, start_topic_name, 1)
        if stop_topic_name is not None:
            self._stop_pub = self.node.create_publisher(String, stop_topic_name, 1)
        if service_topic_name is not None:
            self._srv_pub = self.node.create_publisher(String, service_topic_name, 1)
        self._name = "rosbim_demo_plugin"
        msg = self._name + " initialised."
        self._log(msg, self._init_pub)
        self._config_file_path = config_file_path

        return True

    def _some_callback(
        self, req: Trigger.Request, res: Trigger.Response
    ) -> Trigger.Response:
        ## Callback for service server requests: Outputs a message and publishes to a topic
        #
        #  @param req
        #    The service request
        #  @return
        #    The service response

        msg = self._name + " service called."

        res.success = True
        res.message = msg
        self._log(msg, self._srv_pub)
        return res

    def _start(self) -> bool:
        ## Starts the service server and outputs to console as well as to the given publisher
        #
        #  @return
        #    True if the plug-in was started successfully, else false is returned

        msg = self._name + " started."
        self._log(msg, self._start_pub)

        if self._service_name is not None:
            print("test")
            self._srv = self.node.create_service(
                Trigger, self._service_name, self._some_callback
            )

        return True

    def _stop(self) -> bool:
        ## Stops the service server and outputs to console as well as to the given publisher
        #
        #  @return
        #    True if the plug-in was stopped successfully, else false is returned

        msg = self._name + " stopped."
        self._log(msg, self._stop_pub)

        if self._srv is not None:
            self.node.destroy_service(self._srv)
        return True
