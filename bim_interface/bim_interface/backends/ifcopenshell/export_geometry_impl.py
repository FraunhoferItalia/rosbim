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
from pathlib import Path
import re
import subprocess
from typing import List

import ifcopenshell
import ifcopenshell.geom

from bim_interface.mesh_format import MeshFormat
import ifcopenshell.util.placement
import numpy as np


class ExportGeometry:
    ## Implementation struct of the export geometry plug-in

    # List of attributes name and the predicate in the form of a lambda
    is_a = {
        "Door": (lambda p: p.is_a("IfcDoor")),
        "Furniture": (lambda p: p.is_a("IfcFurniture")),
        "Wall": (lambda p: p.is_a("IfcWall")),
        "Window": (lambda p: p.is_a("IfcWindow")),
    }
    # Attributes that can be returned and a lambda that returns the corresponding encapsulated value
    compare_attributes = {"id": (lambda p: p.id())}
    # Parametrized regular expression for matching a word exactly
    match_exactly = "(?<![A-Za-z]){name}(?![A-Za-z])"
    # Parametrized string that can be used to look up the dictionaries above easily
    replace_str = '{dictionary}["{product_type}"]({product})'

    def __init__(
        self,
        ifc_model: ifcopenshell.file,
        ifc_settings: ifcopenshell.geom.main.settings = ifcopenshell.geom.settings(),
    ) -> None:
        ## Class constructor
        #
        #  @param ifc_model
        #    The IFC model whose geometry should be exported
        #  @param ifc_settings
        #    The IFC geometry settings to be used throughout the process

        self._settings = ifc_settings
        self._ifc_model = ifc_model

        return

    def export(
        self,
        export_dir: str,
        export_file_name: str,
        geometry_filter: str = "",
        is_export_separately: bool = False,
        export_format: MeshFormat = MeshFormat.OBJ,
    ) -> List[str]:
        ## Export a map of the current BIM model and returns the file name where yaml and image files of
        #  the map are located
        #  Uses IfcOpenShell's IfcConvert for creating the geometry
        #
        #  @param export_dir
        #    Directory that the file should be exported to
        #  @param export_file_name
        #    File name of the file to be exported
        #  @param geometry_filter
        #    The query string used to filter out individual geometric elements
        #  @param is_export_separately
        #    Boolean flag indicating whether the individual geometric elements should be exported to
        #    individual files or to a single file instead
        #  @param export_format
        #    Mesh file format the the geometry should be exported in
        #  @return
        #    The paths where the exported geometries are located

        if self._ifc_model is None:
            return None

        # Filter products or export all products
        products = self._ifc_model.by_type("IfcProduct")

        # Create a search query by replacing the string with a call to the corresponding dictionary
        # containing all our lambdas to be evaluated
        modified_filter = '(not (p.is_a("IfcOpeningElement") or p.is_a("IfcSite")) and (p.Representation \
      is not None))'
        if geometry_filter:
            modified_filter = geometry_filter
            for key, value in self.is_a.items():
                modified_filter = re.sub(
                    self.match_exactly.format(name=key),
                    self.replace_str.format(
                        dictionary="self.is_a", product_type=key, product="p"
                    ),
                    modified_filter,
                )  # Must be modified if dictionary is renamed
            for key, value in self.compare_attributes.items():
                modified_filter = re.sub(
                    self.match_exactly.format(name=key),
                    self.replace_str.format(
                        dictionary="self.compare_attributes",
                        product_type=key,
                        product="p",
                    ),
                    modified_filter,
                )

        file_paths = []
        marker_positions = []
        marker_scales = []
        open(os.path.join(export_dir, "marker_scales.txt"), "w").close()
        open(os.path.join(export_dir, "marker_positions.txt"), "w").close()

        if is_export_separately:
            for p in products:  # Variable name p is connected to filter
                if eval(modified_filter):
                    extracted_ifc = ifcopenshell.file(schema="IFC2X3")
                    extracted_ifc.add(p)

                    matrix = ifcopenshell.util.placement.get_local_placement(
                        p.ObjectPlacement
                    )

                    """
          ### overallwidht and overallheight don't work with walls

          marker_scales = []
          marker_scales.append(p.OverallWidth)
          marker_scales.append(0.20)
          marker_scales.append(p.OverallHeight)
          file = open(os.path.join(export_dir, "marker_scales.txt"), "a")
          file.write(str(p.OverallWidth)+" ")
          file.write(str(0.20)+ " ")
          file.write(str(p.OverallHeight) + "\n")
          file.close()
          """

                    print(matrix)
                    m = np.asarray(matrix)
                    rot = m[0:3, 0:3]
                    print(rot)
                    trans = m[0:3, 3]
                    print(trans)

                    position_x = round((trans[0] * 0.001), 2)
                    position_y = round((trans[1] * 0.001), 2)
                    position_z = round((trans[2] * 0.001), 2)

                    file = open(os.path.join(export_dir, str(p.id()) + ".txt"), "w")
                    file.write(str(position_x) + ", ")
                    file.write(str(position_y) + ", ")
                    file.write(str(position_z))
                    file.close()

                    """
          ###### needed for clear paper
           
          marker_positions = []
          marker_positions.append(position_x)
          marker_positions.append(position_y)
          marker_positions.append(position_z)

          file = open(os.path.join(export_dir, "marker_positions.txt"), "a")
          file.write(str(position_x)+" ")
          file.write(str(position_y)+" ")
          file.write(str(position_z) + "\n")
          file.close()
          """

                    file_path = self._save_geometry(
                        extracted_ifc, export_dir, str(p.id()), export_format
                    )
                    file_paths.append(file_path)

        else:
            extracted_ifc = ifcopenshell.file(schema="IFC2X3")
            for p in products:
                if eval(modified_filter):
                    extracted_ifc.add(p)
            file_path = self._save_geometry(
                extracted_ifc, export_dir, export_file_name, export_format
            )
            file_paths.append(file_path)

        return file_paths

    @staticmethod
    def _save_geometry(
        ifc_model: ifcopenshell.file,
        export_dir: str,
        export_file_name: str,
        export_format: MeshFormat = MeshFormat.OBJ,
    ) -> str:
        ## Save the geometry to a mesh file by converting an IFC file using IfcConvert
        #
        #  @param ifc_model
        #    The IFC model whose geometry should be exported
        #  @param export_dir
        #    Directory that the file should be exported to
        #  @param export_file_name
        #    File name of the file to be exported
        #  @param export_format
        #    Mesh file format the the geometry should be exported in
        #  @return
        #    The path where the exported model is located

        extracted_ifc_path = os.path.join(export_dir, export_file_name)
        if not os.path.exists(export_dir):
            os.mkdir(export_dir)
        ifc_model.write(extracted_ifc_path)

        # Convert to other format if necessary
        if export_format == MeshFormat.DAE:
            extracted_dae_path = str(Path(extracted_ifc_path).with_suffix(".dae"))
            extracted_ifc_dir = str(Path(extracted_ifc_path).parent)
            extracted_ifc_name = str(Path(extracted_ifc_path).name)
            # Call IfcConvert: -y for overwriting old files automatically
            # cwd due to bug with mounting in Docker https://github.com/bimspot/xeokit-converter/issues/2
            subprocess.run(
                ["IfcConvert", extracted_ifc_name, extracted_dae_path, "-y"],
                check=True,
                cwd=extracted_ifc_dir,
            )
            return extracted_dae_path
        else:
            extracted_obj_path = str(Path(extracted_ifc_path).with_suffix(".obj"))
            # Call IfcConvert: -y for overwriting old files automatically
            subprocess.run(
                ["IfcConvert", extracted_ifc_path, extracted_obj_path, "-y"], check=True
            )

            if export_format == MeshFormat.OBJ:
                return extracted_obj_path
            else:
                converted_model_path = str(os.path.splitext(extracted_obj_path)[0])
                return ExportGeometry._convert_mesh_file(
                    extracted_obj_path, converted_model_path, export_format
                )

    @staticmethod
    def _convert_mesh_file(
        obj_model_path: str, converted_model_path: str, export_format: MeshFormat
    ) -> str:
        ## Convert the geometry mesh file to a format of your choice if exported to any other format
        #  different from *.obj
        #
        #  @param obj_model_path
        #    The initial geometry mesh file in the *.obj format
        #  @param converted_model_path
        #    File path where the converted model should be saved (with- or without its file ending)
        #  @param export_format
        #    Mesh file format the the geometry should be exported in
        #  @return
        #    The path where the converted model is located

        if export_format == MeshFormat.OBJ:
            return obj_model_path
        else:
            if export_format == MeshFormat.DAE:
                converted_model_path = str(
                    Path(converted_model_path).with_suffix(".dae")
                )
            elif export_format == MeshFormat.STL:
                converted_model_path = str(
                    Path(converted_model_path).with_suffix(".stl")
                )
            else:
                raise NotImplementedError("Conversion not implemented!")
            subprocess.run(
                ["ctmconv", obj_model_path, converted_model_path], check=True
            )
            return converted_model_path
