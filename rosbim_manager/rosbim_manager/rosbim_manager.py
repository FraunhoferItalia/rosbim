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

import argparse
import os
import yaml
import time
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from rclpy.executors import MultiThreadedExecutor
from rcl_interfaces.msg import ParameterType, ParameterDescriptor
from ament_index_python.packages import get_package_share_directory

from bim_interface.plugin_manager import BackendType, PluginManager
from rosbim_manager_msgs.msg import Plugin
from rosbim_manager_msgs.msg import PluginState
from rosbim_manager_msgs.srv import ListPlugins
from rosbim_manager_msgs.srv import LoadBimFile
from rosbim_manager_msgs.srv import LoadPlugin
from rosbim_manager_msgs.srv import SpawnPlugin
from rosbim_manager_msgs.srv import StartPlugin
from rosbim_manager_msgs.srv import StopPlugin
from rosbim_manager_msgs.srv import SwitchBackend
from rosbim_manager_msgs.srv import UnloadPlugin


class RosbimManager(PluginManager):
    ## Plugin manager for the ROSBIM package

    def __init__(
        self,
        bim_file: str = None,
        backend_type: BackendType = BackendType.IFC_OPEN_SHELL,
    ) -> None:
        ## Class constructor: Registers services
        #
        #  @param bim_file
        #    BIM file to be used by upon start
        #  @param backend_type
        #    Backend to be used upon start

        self.node = rclpy.create_node("rosbim_manager")

        # Get the parameter value by name
        bim_file_name = "bim_file"
        bim_file_param_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_STRING, description="The path to the bim file."
        )
        self.node.declare_parameter("bim_file", "", bim_file_param_descriptor)

        # Check if the parameter exists
        bim_file = ""
        if self.node.has_parameter(bim_file_name):
            bim_file = self.node.get_parameter(bim_file_name).value
            self.node.get_logger().info(f"Loading the  {bim_file_name}: {bim_file}")
        else:
            self.node.get_logger().error(
                f"Parameter {bim_file_name} does not exist. Exiting..."
            )
            return

        PluginManager.__init__(self, bim_file, backend_type)

        self._switch_backend_srv = self.node.create_service(
            SwitchBackend, "~/switch_backend", self._switch_backend_cbck
        )
        self._load_bim_file_srv = self.node.create_service(
            LoadBimFile, "~/load_bim_file", self._load_bim_file_cbck
        )

        self._clear_bim_file_srv = self.node.create_service(
            Trigger, "~/clear_bim_file", self._clear_bim_file_cbck
        )

        self._kill_plugins_srv = self.node.create_service(
            Trigger, "~/kill_plugins", self._kill_plugins_cbck
        )
        self._list_plugins_srv = self.node.create_service(
            ListPlugins, "~/list_plugins", self._list_plugins_cbck
        )
        self._load_plugin_srv = self.node.create_service(
            LoadPlugin, "~/load_plugin", self._load_plugin_cbck
        )
        self._spawn_plugin_srv = self.node.create_service(
            SpawnPlugin, "~/spawn_plugin", self._spawn_plugin_cbck
        )
        self._start_plugin_srv = self.node.create_service(
            StartPlugin, "~/start_plugin", self._start_plugin_cbck
        )
        self._stop_plugin_srv = self.node.create_service(
            StopPlugin, "~/stop_plugin", self._stop_plugin_cbck
        )
        self._unload_plugin_srv = self.node.create_service(
            UnloadPlugin, "~/unload_plugin", self._unload_plugin_cbck
        )
        self.node.get_logger().info("ROSBIM manager started...")

        self.executor = MultiThreadedExecutor(8)
        self.executor.add_node(self.node)
        self._loop()
        return

    def _switch_backend_cbck(
        self, request: SwitchBackend.Request, response: SwitchBackend.Response
    ):
        ## Callback for switching the BIM backend
        #
        #  @param req
        #    The service request
        #  @return
        #    The service response

        self.node.get_logger().info("Switching backend...")
        response = SwitchBackend.Response()
        try:
            response.success = self.switch_backend(BackendType[request.backend_type])
        except Exception as ex:
            response.success = False
            self.node.get_logger().error(
                "Failed to load back-end '"
                + request.backend_type
                + "': Reason: '"
                + str(ex)
                + "'."
            )
        return response

    def _load_bim_file_cbck(
        self, request: LoadBimFile.Request, response: LoadBimFile.Response
    ):
        ## Callback for loading BIM file
        #
        #  @param req
        #    The service request
        #  @return
        #    The service response

        self.node.get_logger().info("Loading BIM file...")
        response = LoadBimFile.Response()
        try:
            response.success = self.load_bim_file(request.file_path)
        except Exception as ex:
            response.success = False
            self.node.get_logger().error(
                "Failed to load BIM file '"
                + request.file_path
                + "': Reason: '"
                + str(ex)
                + "'."
            )
        return response

    def _clear_bim_file_cbck(
        self, request: Trigger.Request, response: Trigger.Response
    ):
        ## Callback for clearing loaded BIM file
        #
        #  @param req
        #    The service request
        #  @return
        #    The service response

        self.node.get_logger().info("Clearing BIM file...")
        response = Trigger.Response()
        response.success = self.clear_bim_file()
        return response

    def _kill_plugins_cbck(self, request: Trigger.Request, response: Trigger.Response):
        ## Callback for killing all loaded plug-ins
        #
        #  @param req
        #    The service request
        #  @return
        #    The service response

        self.node.get_logger().info("Killing all loaded plug-ins...")
        response = Trigger.Response()
        self._remove_plugin_nodes_from_executor()

        self.kill_all_plugins()
        response.success = True

        return response

    def _list_plugins_cbck(
        self, request: ListPlugins.Request, response: ListPlugins.Response
    ):
        ## Callback for listing all plug-ins and its status
        #
        #  @param req
        #    The service request
        #  @return
        #    The service response

        response = ListPlugins.Response()
        for plugin_name in self._plugins:
            config_path, state = self.info(plugin_name)
            plugin_state = PluginState()
            plugin_state.plugin.package_name = plugin_name
            plugin_state.plugin.config_file = config_path
            plugin_state.state = state
            response.plugins.append(plugin_state)
        return response

    def _load_plugin_cbck(
        self, request: LoadPlugin.Request, response: LoadPlugin.Response
    ):
        ## Callback for loading a specific plug-in
        #
        #  @param req
        #    The service request
        #  @return
        #    The service response

        self.node.get_logger().info(
            "Loading plug-in '" + request.plugin.package_name + "'..."
        )
        res = LoadPlugin.Response()
        # Convention: Inside package src folder, folder with package name and finally script with same name ending in .py
        module_name = request.plugin.package_name
        try:
            rospack = rospkg.RosPack()
            module_name = request.plugin.package_name
            if module_name.startswith("rosbim_"):
                module_name = module_name[len("rosbim_") :]
            module_path = os.path.join(
                rospack.get_path(request.plugin.package_name),
                "src",
                request.plugin.package_name,
                module_name + ".py",
            )
            config_file = None
            if request.plugin.config_file:
                config_file = request.plugin.config_file
            plugin_names = self.load_plugin(module_name, module_path, config_file)
            if plugin_names:
                res.success = True
            else:
                res.success = False
        except Exception as e:
            self.node.get_logger().error(
                "Failed to load plug-in '"
                + request.plugin.package_name
                + "': Reason '"
                + str(e)
                + "'."
            )
            res.success = False

        return res

    def _spawn_plugin_cbck(
        self, request: SpawnPlugin.Request, response: SpawnPlugin.Response
    ):
        ## Callback for spawning (loading + starting) a specifc plug-in
        #
        #  @param req
        #    The service request
        #  @return
        #    The service response

        self.node.get_logger().info(
            "Spawning plug-in '" + request.plugin.package_name + "'..."
        )
        response = SpawnPlugin.Response()
        module_name = request.plugin.package_name
        plugin_name = ""
        try:
            module_name = request.plugin.package_name
            if module_name.startswith("rosbim_"):
                module_name = module_name[len("rosbim_") :]
            module_path = os.path.join(
                get_package_share_directory(request.plugin.package_name),
                module_name + ".py",
            )
            self.node.get_logger().info(module_path)
            config_file = None
            if request.plugin.config_file:
                config_file = request.plugin.config_file

            response.success, plugin_name = self.spawn_plugin(
                module_name, module_path, config_file
            )

            if not response.success:
                self.node.get_logger().error(
                    "Failed to spawn plug-in " + request.plugin.package_name
                )
        except Exception as e:
            self.node.get_logger().error(
                "Failed to load plug-in '"
                + request.plugin.package_name
                + "': Reason '"
                + str(e)
                + "'."
            )
            response.success = False
        if plugin_name != "":
            self._add_plugin_node_to_executor(plugin_name)

        return response

    def _start_plugin_cbck(
        self, request: StartPlugin.Request, response: StartPlugin.Response
    ):
        ## Callback for starting a specific plug-in
        #
        #  @param req
        #    The service request
        #  @return
        #    The service response

        self.node.get_logger().info(
            "Starting plug-in '" + request.package_name + "'..."
        )
        response = StartPlugin.Response()
        response.success = self.start_plugin(request.package_name)
        return response

    def _stop_plugin_cbck(
        self, request: StopPlugin.Request, response: StopPlugin.Response
    ):
        ## Callback for stopping a specific plug-in
        #
        #  @param request
        #    The service request
        #  @return response
        #    The service response

        self.node.get_logger().info(
            "Stopping plug-in '" + request.package_name + "'..."
        )
        response = StopPlugin.Response()
        response.success = self.stop_plugin(request.package_name)
        return response

    def _unload_plugin_cbck(
        self, request: UnloadPlugin.Request, response: UnloadPlugin.Response
    ):
        ## Callback for unloading a specific plug-in
        #
        #  @param request
        #    The service request
        #  @param response
        #    The service response

        self.node.get_logger().info(
            "Unloading plug-in '" + request.package_name + "'..."
        )
        response = UnloadPlugin.Response()
        self._remove_plugin_node_from_executor(request.package_name)
        response.success = self.unload_plugin(request.package_name)
        return response

    def _loop(self):
        while True:
            self.executor.spin_once()
            time.sleep(0.1)

    def _add_plugin_node_to_executor(self, plugin_name):
        self.executor.add_node(self._plugins[plugin_name].node)
        return

    def _remove_plugin_node_from_executor(self, plugin_name):
        return self.executor.remove_node(self._plugins[plugin_name].node)

    def _remove_plugin_nodes_from_executor(self):
        for plugin_name in self._plugins:
            return self.executor.remove_node(self._plugins[plugin_name].node)


def main(args=None):
    ## Main loop: Parses the input arguments, instantiates the RosbimManager and lets it spin
    rclpy.init(args=args)

    manager = RosbimManager()
    rclpy.shutdown()
    print("Shutting down")


if __name__ == "__main__":
    main()
