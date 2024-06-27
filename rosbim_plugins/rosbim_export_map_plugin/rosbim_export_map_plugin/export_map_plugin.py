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
import wrapt

import rclpy

from rosbim_manager_msgs.msg import PluginState
from bim_interface.plugin_base import PluginBase, State
from bim_interface.backend import _backend, _backend_lock
from rosbim_export_map_plugin.srv import ExportMapService
from ament_index_python.packages import get_package_share_directory


class ExportMap(PluginBase):
    ## Plug-in class to extract map from the selected BIM file

    def _init(
        self,
        config_file_path: str = os.path.join(
            get_package_share_directory("rosbim_export_map_plugin"),
            "config",
            "config_file.yaml",
        ),
    ) -> bool:
        ## Initialise the class members
        #  @param config_file_path
        #    The path of the package configuration file
        #  @return
        #    True if the plug-in was unloaded successfully, else false is returned
        #  @return
        #    True if the plug-in was initialized successfully, else false is returned
        self.node.get_logger().info(config_file_path)
        self._export_map_srv = None
        return True

    @wrapt.synchronized(_backend_lock)
    def _create_map(
        self, req: ExportMapService.Request, res: ExportMapService.Response
    ):  # MsgType._request_class  # -> MsgType._response_class
        ## Callback for service server requests: Outputs a message with the map path
        #
        #  @param req
        #    The service request
        #  @return
        #    The service response

        global _backend
        (map_file_path, yaml_file_path) = _backend.export_map(
            req.section_heights, req.resolution
        )

        if map_file_path is not None:
            res.success = True
            res.map_file = map_file_path
            res.config_file = yaml_file_path
        else:
            res.success = False
        return res

    def _start(self) -> bool:
        ## Starts the service server and outputs to console
        #
        #  @return
        #    True if the plug-in was started successfully, else false is returned

        self._export_map_srv = self.node.create_service(
            ExportMapService, "~/export_map", self._create_map
        )
        return True

    def _stop(self) -> bool:
        ## Stops the service server and outputs to console
        #
        #  @return
        #    True if the plug-in was stopped successfully, else false is returned

        if self._export_map_srv is not None:
            self.node.destroy_service(self._export_map_srv)
        return True
