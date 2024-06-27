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
from rclpy.node import Node

from bim_interface.mesh_format import MeshFormat
from bim_interface.plugin_base import PluginBase, State
from bim_interface.backend import _backend, _backend_lock
from rosbim_manager_msgs.msg import PluginState
from rosbim_export_geometry_plugin.srv import ExportGeometryService

from ament_index_python.packages import get_package_share_directory


class ExportGeometryPlugin(PluginBase):
    ## Plug-in class to extract geometries from the selected BIM file

    def _init(
        self,
        config_file_path: str = os.path.join(
            get_package_share_directory("rosbim_export_geometry_plugin"),
            "config",
            "config_file.yaml",
        ),
    ) -> bool:
        ## Initialise the class members
        #
        #  @param config_file_path
        #    The path of the package configuration file
        #  @return
        #    True if the plug-in was unloaded successfully, else false is returned
        #  @return
        #    True if the plug-in was initialized successfully, else false is returned

        self.node.get_logger().info(config_file_path)
        self._geometry_export_srv = None

        return True

    @wrapt.synchronized(_backend_lock)
    def _extract_geometry(
        self, req: ExportGeometryService.Request, res: ExportGeometryService.Response
    ):  # -> MsgType._response_class
        ## Callback for service server requests: Outputs a message with the extracted geometry path
        #
        #  @param req
        #    The service request
        #  @return
        #    The service response
        global _backend

        geometry_filter = req.geometry_filter
        is_export_separately = req.is_export_separately
        export_format = MeshFormat(req.export_format)

        try:
            export_file_paths = _backend.export_geometry(
                geometry_filter, is_export_separately, export_format
            )
        except Exception as e:
            self.node.get_logger().error(e)

        if export_file_paths is not None:
            res.success = True
            res.geometry_files = export_file_paths
        else:
            res.success = False
        return res

    def _start(self) -> bool:
        ## Starts the service server and outputs to console
        #
        #  @return
        #    True if the plug-in was started successfully, else false is returned
        try:
            self._geometry_export_srv = self.node.create_service(
                ExportGeometryService, "~/export_geometry", self._extract_geometry
            )
        except Exception as e:
            print(e)

        return True

    def _stop(self) -> bool:
        ## Stops the service server and outputs to console
        #
        #  @return
        #    True if the plug-in was stopped successfully, else false is returned

        # if self._geometry_export_srv is not None:
        #    self.node.destroy_service(self._geometry_export_srv)
        return True
