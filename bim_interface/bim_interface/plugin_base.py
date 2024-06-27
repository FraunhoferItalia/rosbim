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

# Base plugin for all ROSBIM plug-ins: All ROSBIM plug-ins should inherit from this class.

import abc
from enum import Enum

from bim_interface.bim_interface import BimInterface

import rclpy
from rclpy.node import Node


class State(Enum):
    ## Enum for all the individual states

    LOADED = 0
    RUNNING = 1
    STOPPED = 2


class PluginBase(metaclass=abc.ABCMeta):
    ## The pure abstract base class for all ROSBIM plug-ins

    def __init__(self):
        ## Class constructor
        self._state = None
        self._config_file_path = None

    def init(self, config_file_path: str = None, node_name: str = None) -> None:
        ## The method called upon initialisation of the plug-in, forwards to the implementation _init
        #
        #  @param config_file_path
        #    The file path of the configuration file to be parsed

        self.node = rclpy.create_node(node_name)
        self._config_file_path = config_file_path
        is_success = False
        if config_file_path is not None:
            is_success = self._init(config_file_path)
        else:
            is_success = self._init()
        if is_success:
            self._state = State.LOADED
        return None

    def start(self) -> None:
        ## The method called when starting the plug-in, forwards to implementation _start
        is_success = self._start()
        if is_success:
            self._state = State.RUNNING
        return None

    def stop(self) -> None:
        ## The method called when stopping the plug-in, forwards to implementation _stop

        is_success = self._stop()
        if is_success:
            self._state = State.STOPPED
        return None

    def getConfigPath(self) -> str:
        ## Method for getting the path of the configuration file currently in use
        #
        #  @return
        #    Name of the config file currently used

        if self._config_file_path is not None:
            return self._config_file_path
        else:
            return "<default>"

    def getState(self) -> State:
        ## Method for getting current status of plug-in
        #
        #  @return
        #    State of the plug-in

        return self._state

    def getType(self) -> str:
        ## Method for getting the type of the derived object
        #
        #  @return
        #    Name of the class

        return self.__class__.__name__

    def init_executor(self, executor):
        ## The method to spin the Node
        #
        #  @warning
        #    Must be implemented by the derived class!
        executor.add_node(self.node)
        return executor

    @abc.abstractmethod
    def _init(self, config_file_path: str = None) -> bool:
        ## The method called upon initialisation
        #
        #  @warning
        #    Must be implemented by the derived class!
        #  @param config_file_path
        #    The file path of the configuration file to be parsed
        #  @return
        #    True if the plug-in was initialized successfully, else false is returned

        pass

    @abc.abstractmethod
    def _start(self) -> bool:
        ## The method called when starting the plug-in
        #
        #  @warning
        #    Must be implemented by the derived class!
        #  @return
        #    True if the plug-in was started successfully, else false is returned

        pass

    @abc.abstractmethod
    def _stop(self) -> bool:
        ## The method called when stopping the plug-in
        #
        #  @warning
        #    Must be implemented by the derived class!
        #  @return
        #    True if the plug-in was stopped successfully, else false is returned

        pass
