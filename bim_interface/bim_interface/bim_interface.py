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


import abc
from enum import Enum
from typing import List, Tuple

from bim_interface.mesh_format import MeshFormat


class BimInterface:
    ## The pure abstract base class for all ROSBIM backends

    def __init__(self, bim_model_path: str):
        ## Class constructor
        #
        #  @param bim_model_path
        #    Path of the BIM file to be read

        self._bim_model_path = bim_model_path
        return

    @abc.abstractmethod
    def export_geometry(
        self,
        geometry_filter: str = "",
        is_export_separately: bool = False,
        export_format: MeshFormat = MeshFormat.OBJ,
    ) -> List[str]:
        ## Export the current BIM model to a file and returns the file name where the geometry is located
        #
        #  @param geometry_filter
        #    The query string used to filter out individual geometric elements
        #  @param is_export_separately
        #    Boolean flag indicating whether the individual geometric elements should be exported to
        #    individual files or to a single file instead
        #  @param export_format
        #    Mesh file format the the geometry should be exported in
        #  @return
        #    The paths where the exported geometries are located

        pass

    @abc.abstractmethod
    def export_map(
        self, cross_section_heights: List[float], resolution: float
    ) -> Tuple[str, str]:
        ## Export a map of the current BIM model and returns the file name where yaml and image files of the map are located
        #
        #  @param cross_section_heights
        #    Heights at which the map should be obtained, if more than one heights are given the different
        #    Cross-sections are merged additively to a single map by projection from above
        #  @param resolution
        #    Resolution of the map in [meters/pixel]
        #  @return
        #    The path where the image and yaml files are located

        pass
