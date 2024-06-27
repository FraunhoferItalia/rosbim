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
from typing import List, Tuple

import ifcopenshell
import ifcopenshell.geom

from bim_interface.bim_interface import BimInterface, MeshFormat


class IfcOpenShell(BimInterface):
    ## BIM backend based on IfcOpenShell

    def __init__(self, bim_model_path: str) -> None:
        ## Class constructor
        #
        #  @param bim_model_path
        #    Path of the BIM file to be read

        super().__init__(bim_model_path)
        self._settings = ifcopenshell.geom.settings()
        self._ifc_model = None
        self._export_file_name = (
            None  # Set inside the open file menu depending on the file location
        )
        self._export_dir = None  # This value too

        if bim_model_path is not None:
            self.open_file(bim_model_path)

    def open_file(self, bim_model_path) -> None:
        ## Open a local BIM file
        #
        #  @param bim_model_path
        #    Path of the BIM file to be read

        self._bim_model_path = bim_model_path
        self._ifc_model = ifcopenshell.open(self._bim_model_path)
        (file_dir, file_name) = os.path.split(self._bim_model_path)
        self._export_file_name = os.path.splitext(file_name)[0]
        self._export_dir = os.path.join(
            file_dir, "temp"
        )  # Create temporary folder inside the original BIM file path
        if not os.path.exists(self._export_dir):
            os.mkdir(self._export_dir)
        return

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

        from bim_interface.backends.ifcopenshell.export_geometry_impl import (
            ExportGeometry,
        )

        if self._ifc_model is None:
            return None
        impl = ExportGeometry(self._ifc_model, self._settings)
        return impl.export(
            self._export_dir,
            self._export_file_name,
            geometry_filter,
            is_export_separately,
            export_format,
        )

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

        from bim_interface.backends.ifcopenshell.export_map_impl import ExportMap

        if self._ifc_model is None:
            return None
        impl = ExportMap(self._ifc_model, self._settings)
        return impl.export(
            self._export_dir, self._export_file_name, cross_section_heights, resolution
        )
