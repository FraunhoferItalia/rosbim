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
from enum import Enum
from importlib.machinery import SourceFileLoader
import inspect
import os
import re
from typing import Dict, Tuple
import yaml


from .backends.ifcopenshell.ifcopenshell import IfcOpenShell
from .backend import BackendType, switch_backend
from .bim_interface import BimInterface
from .plugin_base import PluginBase, State


class PluginManager:
    ## Plugin manager for the ROSBIM package without any ROS functionality

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

        self._plugins = dict()  # Dictionary of plug-in name and plug-in instance
        self._bim_file = bim_file
        self._backend_type = backend_type

        self._update_backend()

    def _update_backend(self) -> bool:
        ## Updates the backend BIM library with the BIM file and desired backend by instantiating a new backend
        #
        #  @return
        #    True if the backend was updated successfully, else false
        print(self._bim_file)
        return switch_backend(self._backend_type, self._bim_file)

    def switch_backend(self, backend_type: BackendType) -> bool:
        ## Set the backend of the application
        #
        #  @return
        #    True if the backend was loaded successfully, else false

        self._backend_type = backend_type
        self._update_backend()
        return False

    def load_bim_file(self, bim_file: str) -> bool:
        ## Set the BIM file to be processed
        #
        #  @param bim_file
        #    The BIM file to be loaded
        #  @return
        #    True if the BIM file was loaded successfully, else false

        self._bim_file = bim_file
        self._update_backend()
        global _backend
        print("test")
        return False

    def clear_bim_file(self) -> bool:
        ## Clear the currently set BIM file
        #  @return
        #    True if the BIM file was cleared successfully, else false

        self._bim_file = None
        return True

    def _discover_plugins(
        self, module_name: str, module_path: str
    ) -> Dict[str, PluginBase]:
        ## Function for discovering plug-ins inside a given Python module
        #
        #  @param module_name
        #    Name of the Python module to be searched for plug-ins
        #  @param module_path
        #    Path of the Python module to be searched for plug-ins including the file and ending itself
        #  @return
        #    A dictionary of discovered plug-ins, if not found the dictionary is empty
        print("Discovering plugins... ")
        plugins_found = dict()
        try:
            mod = SourceFileLoader(module_name, module_path).load_module()
            mod_classes = dict(
                [
                    (name, cls)
                    for name, cls in mod.__dict__.items()
                    if isinstance(cls, type)
                ]
            )
            for class_type, c in mod_classes.items():
                base_classes = inspect.getmro(c)
                is_plugin = (PluginBase.__name__ in str(base_classes)) and (
                    str(base_classes).count("class") > 2
                )
                # print(class_type)
                if is_plugin:
                    plugins_found[class_type] = c()
        except Exception as e:
            print(e)
            pass
        return plugins_found

    def load_plugin(
        self, module_name: str, module_path: str, config_file_path: str = None
    ) -> str:
        ## Function for loading a specific plug-in
        #
        #  @param module_name
        #    Name of the Python module to be searched for plug-ins
        #  @param module_path
        #    Path of the Python module to be searched for plug-ins including the file and ending itself
        #  @param config_file_path
        #    File path of the configuration file if given, else defaults to default argument
        #  @return
        #    Returns the plugin_name that the plug-in was loaded to. This difference is necessary to allow
        #    different plug-ins of the same type
        print("Loading plugin: " + module_name)
        plugins = self._discover_plugins(module_name, module_path)
        plugin_names = list()
        for plugin_type, plugin in plugins.items():
            plugin_count = -1
            for other_plugin_name in self._plugins:
                other_count = re.match(
                    plugin_type + "_(\d+)", other_plugin_name, re.IGNORECASE
                )
                if other_count is not None:
                    plugin_count = max(plugin_count, int(other_count.group(1)))
            plugin_name = plugin_type + "_" + str(plugin_count + 1)
            plugin_names.append(plugin_name)
            self._plugins[plugin_name] = plugin
            node_name = module_name + "_node"
            self._plugins[plugin_name].init(config_file_path, node_name)

        return plugin_names

    def start_plugin(self, plugin_name: str) -> bool:
        ## Function for starting a specific plug-in
        #
        #  @param plugin_name
        #    Name of the package containing the ROSBIM plug-in
        #  @return
        #    Boolean argument signaling whether the plug-in could be loaded or not

        if plugin_name in self._plugins:
            plugin = self._plugins[plugin_name]

            if plugin.getState() is not State.RUNNING:
                print(plugin_name)
                plugin.start()
                return True
        return False

    def spawn_plugin(
        self, module_name: str, module_path: str, config_file_path: str = None
    ) -> bool:
        ## Function for spawning (loading + starting) a specific plug-in
        #
        #  @param module_name
        #    Name of the Python module to be searched for plug-ins
        #  @param module_path
        #    Path of the Python module to be searched for plug-ins including the file and ending itself
        #  @param config_file_path
        #    File path of the configuration file if given, else defaults to default argument
        #  @return
        #    Boolean argument signaling whether the plug-in was spawned successfully (true) or not (false)
        print("Spawing plugin: " + module_name)
        try:
            plugins = self.load_plugin(module_name, module_path, config_file_path)
            is_success = True
            for plugin_name in plugins:
                is_success = self.start_plugin(plugin_name) and is_success
                if is_success:
                    print("Successfully spawned plugin: " + plugin_name)

            return is_success, plugin_name
        except:
            return False, ""

    def info(self, plugin_name: str) -> Tuple[str, str]:
        ## Returns information about the configuration file being used as well as the status of the plug-in
        #
        #  @param plugin_name
        #    Name of the package containing the ROSBIM plugin whose the info should be shown
        #  @return
        #    A tuple containing the configuration path and its state

        if plugin_name in self._plugins:
            plugin = self._plugins[plugin_name]
            config_path = plugin.getConfigPath()
            state = plugin.getState()
            return (config_path, state.name)
        else:
            raise Exception("Plugin '" + plugin_name + "' not loaded!")

    def list_plugins(self) -> Dict[str, Tuple[str, str]]:
        ## Function for listing all plug-ins and their status
        #
        #  @param plugin_name
        #    Name of the package containing the ROSBIM plug-in
        #  @return
        #    A list of all plugins with their used configuration file directory and state

        plugins = dict()
        for plugin_name in self._plugins:
            plugins[plugin_name] = self.info(plugin_name)
        return plugins

    def stop_plugin(self, plugin_name: str) -> bool:
        ## Function for stopping a specific plug-in
        #
        #  @param plugin_name
        #    Name of the package containing the ROSBIM plug-in
        #  @return
        #    Boolean argument signaling whether the plug-in was stopped successfully (true) or not (false)

        if plugin_name in self._plugins:
            plugin = self._plugins[plugin_name]
            if plugin.getState() is State.RUNNING:
                plugin.stop()
                return True
        return False

    def unload_plugin(self, plugin_name: str) -> bool:
        ## Function for unloading a specific plug-in
        #
        #  @param plugin_name
        #    Name of the package containing the ROSBIM plug-in
        #  @return
        #    Boolean argument signaling whether the plug-in was unloaded successfully (true) or not (false)

        if plugin_name in self._plugins:
            plugin = self._plugins[plugin_name]
            state = plugin.getState()
            if plugin.getState() is State.RUNNING:
                return False
            if state is State.LOADED or state is State.STOPPED:
                del self._plugins[plugin_name]
                return True
        return False

    def kill_plugin(self, plugin_name: str) -> bool:
        ## Function for stopping a specific plug-in
        #
        #  @param plugin_name
        #    Name of the package containing the ROSBIM plug-in
        #  @return
        #    Boolean argument signaling whether the plug-in was stopped successfully (true) or not (false)

        if plugin_name in self._plugins:
            is_stopped = self.stop_plugin(plugin_name)
            is_unloaded = self.unload_plugin(plugin_name)
            return is_stopped and is_unloaded
        return False

    def kill_all_plugins(self) -> bool:
        ## Function for killing all loaded plug-ins
        #
        #  @return
        #    Boolean argument signaling whether the plug-ins were killed successfully (true) or not (false)

        is_success = True
        for plugin_name in list(
            self._plugins.keys()
        ):  # .keys(): Copy required for iteration over dictionary with changing size
            is_success = self.kill_plugin(plugin_name) and is_success

        return is_success

    def add_plugin_node_to_executor(self, executor):
        ## Function for spinning the nodes of the plugins
        #
        #  @return
        #    Boolean argument signaling whether the plug-ins were spin successfully (true) or not (false)
        for plugin_name in self._plugins:
            return executor.add_node(self._plugins[plugin_name].node)
        return
