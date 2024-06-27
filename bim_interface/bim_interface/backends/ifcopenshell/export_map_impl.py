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

# Implementation of the export map plug-in
# This is largely based on the following IfcOpenShell tutorial
# https://academy.ifcopenshell.org/posts/using-ifcopenshell-and-pythonocc-to-generate-cross-sections-directly-from-an-ifc-file/
# For the code documentation see the official documentation at https://dev.opencascade.org/doc/refman/html/index.html


import os
from pathlib import Path
import subprocess
from typing import List, Tuple

import ifcopenshell
import ifcopenshell.geom

# Modules were moved from OCC to OCC.Core
from OCC.Core import (
    Bnd,
    BRepAlgoAPI,
    BRepBndLib,
    BRepBuilderAPI,
    BRepCheck,
    gp,
    Quantity,
    TopAbs,
    TopExp,
    TopTools,
)
from OCC.Display import OCCViewer
from PIL import Image


class ExportMap:
    ## Implementation struct of the export geometry plug-in

    def __init__(
        self,
        ifc_model: ifcopenshell.file,
        ifc_settings: ifcopenshell.geom.main.settings = ifcopenshell.geom.settings(),
    ) -> None:
        ## Class constructor
        #  Saves the Ifc BIM model and the settings inside the class so that they can be accessed later
        #
        #  @param ifc_model
        #    The IFC model whose geometry should be exported
        #  @param ifc_settings
        #    The IFC geometry settings to be used throughout the process

        self._ifc_settings = ifc_settings
        self._ifc_model = ifc_model
        self._ifc_settings.set(self._ifc_settings.USE_PYTHON_OPENCASCADE, True)
        return

    def export(
        self,
        export_dir: str,
        export_file_name: str,
        cross_section_heights: List[float] = [1.0],
        resolution: float = 0.1,
    ) -> str:
        ## Export a map of the current BIM model and returns the file name where yaml-file of the map is located
        #
        #  @param export_dir
        #    Directory that the files (map and *.yaml-file) should be exported to
        #  @param export_file_name
        #    File name of the files (map and *.yaml-file) to be exported without the file ending
        #  @param cross_section_heights
        #    Heights at which the map should be obtained, if more than one heights are given the different
        #    Cross-sections are merged additively to a single map by projection from above
        #  @param resolution
        #    Resolution of the map in [meters/pixel]
        #  @return
        #    The path where the *.yaml configuration file is saved

        # Settings that are not exposed through the API
        background_colour = Quantity.Quantity_Color(Quantity.Quantity_NOC_WHITE)
        wire_colour = Quantity.Quantity_Color(Quantity.Quantity_NOC_BLACK)
        face_colour = wire_colour

        relevant_products = self._extract_products(self._ifc_model)

        os.environ["PYTHONOCC_OFFSCREEN_RENDERER"] = "1"
        occ_display = ifcopenshell.geom.utils.initialize_display()
        occ_display.View.SetBackgroundColor(background_colour)

        bounding_box = Bnd.Bnd_Box()
        for height in cross_section_heights:
            (
                open_wires,
                closed_wires,
                faces,
                cross_section_bounding_box,
            ) = self._generate_cross_section(
                relevant_products, height, self._ifc_settings
            )
            self._draw(occ_display, open_wires, wire_colour)
            self._draw(occ_display, closed_wires, wire_colour)
            self._draw(occ_display, faces, face_colour)
            bounding_box.Add(cross_section_bounding_box)

        map_path = self._save_map(
            occ_display, export_dir, export_file_name, bounding_box, resolution
        )

        bounding_box_dim = bounding_box.Get()
        origin_x = bounding_box_dim[0]
        origin_y = bounding_box_dim[1]
        map_yaml_path = self._save_config_file(
            export_dir,
            export_file_name,
            map_path,
            resolution,
            (origin_x, origin_y, 0.0),
            occupied_threshold=0.65,
            free_threshold=0.196,
            is_negate=False,
        )

        return (map_path, map_yaml_path)

    @staticmethod
    def _extract_products(
        ifc_model: ifcopenshell.file,
    ) -> List[ifcopenshell.entity_instance]:
        ## Filter out the irrelevant product descriptions and only return the ones that have to be drawn
        #
        #  @param ifc_model
        #    The IFC model whose geometry should be exported
        #  @return
        #    A list of the extracted relevant product descriptions

        products = ifc_model.by_type("IfcProduct")
        relevant_products = []
        for p in products:
            if p.is_a("IfcOpeningElement") or p.is_a("IfcSite"):
                continue
            elif p.Representation is not None:
                relevant_products.append(p)
            else:
                # The other object types are neglected
                continue
        return relevant_products

    @staticmethod
    def _generate_cross_section(
        products: List[ifcopenshell.entity_instance],
        cross_section_height: float = 1.0,
        ifc_settings: ifcopenshell.geom.main.settings = ifcopenshell.geom.settings(),
    ) -> Tuple[
        TopTools.TopTools_HSequenceOfShape,
        TopTools.TopTools_HSequenceOfShape,
        TopTools.TopTools_HSequenceOfShape,
        Bnd.Bnd_Box,
    ]:
        ## Create a map of different products by intersecting them with a horizontal cross-section at a
        #  given height
        #
        #  @param products
        #    The products that should be intersected with the plane at the given height
        #  @param cross_section_height
        #    The height (in meters) in absolute coordinates where the cross-section should be taken from
        #  @param ifc_settings
        #    The IFC geometry settings to be used throughout the process
        #  @return
        #    The open and closed wires as well as the faces generated from the closed wires that were
        #    obtained by the intersection of the plane with the corresponding products and the corresponding
        #    bounding box

        model_bounding_box = (
            Bnd.Bnd_Box()
        )  # Bounding box for estimating the dimensions of the horizontal face
        for p in products:
            shape = ifcopenshell.geom.create_shape(ifc_settings, p).geometry
            BRepBndLib.brepbndlib.Add(shape, model_bounding_box)
        model_bounding_box_dim = model_bounding_box.Get()
        model_bounding_box_dx = model_bounding_box_dim[3] - model_bounding_box_dim[0]
        model_bounding_box_dy = model_bounding_box_dim[4] - model_bounding_box_dim[1]
        model_origin_x = model_bounding_box_dim[0]
        model_origin_y = model_bounding_box_dim[1]
        # Minimum and maximum parametric parameter for the cross-section plane to be considered
        u = (
            0.0 - 0.1 * model_bounding_box_dx,
            model_bounding_box_dx + 0.1 * model_bounding_box_dx,
        )
        v = (
            0.0 - 0.1 * model_bounding_box_dy,
            model_bounding_box_dy + 0.1 * model_bounding_box_dy,
        )

        # Create a face with a limited extension from the horizontal plane
        section_plane = gp.gp_Pln(
            gp.gp_Pnt(model_origin_x, model_origin_y, cross_section_height),
            gp.gp_Dir(0, 0, 1),
        )
        section_face = BRepBuilderAPI.BRepBuilderAPI_MakeFace(
            section_plane, u[0], u[1], v[0], v[1]
        ).Face()

        closed_wires = TopTools.TopTools_HSequenceOfShape()
        open_wires = TopTools.TopTools_HSequenceOfShape()
        cross_section_bounding_box = (
            Bnd.Bnd_Box()
        )  # The bounding box of the cross-section might be smaller!
        for product in products:
            shape = ifcopenshell.geom.create_shape(ifc_settings, product).geometry
            section = BRepAlgoAPI.BRepAlgoAPI_Section(section_face, shape).Shape()
            BRepBndLib.brepbndlib.Add(shape, cross_section_bounding_box)

            # Extract the resulting edges
            exp = TopExp.TopExp_Explorer(section, TopAbs.TopAbs_EDGE)
            edges = TopTools.TopTools_HSequenceOfShape()
            while exp.More():
                edges.Append(exp.Current())
                exp.Next()

            while edges.Length() > 0:
                # Create a open or closed wire by combining the different edges
                wire_maker = BRepBuilderAPI.BRepBuilderAPI_MakeWire()
                parent_edge = edges.Value(1)
                wire_maker.Add(parent_edge)
                wire = wire_maker.Wire()
                i = 2
                while i <= edges.Length():
                    child_edge = edges.Value(i)
                    wire_maker.Add(child_edge)
                    if wire_maker.Error():
                        i = i + 1
                    else:
                        # Wire has to be created here and not at the end: The IsDone() method fails in case the
                        # last edge was not added successfully!
                        wire = wire_maker.Wire()
                        edges.Remove(i)
                        i = 2
                check_wire = BRepCheck.BRepCheck_Wire(wire)
                if check_wire.Closed() == BRepCheck.BRepCheck_Status.BRepCheck_NoError:
                    closed_wires.Append(wire)
                else:
                    open_wires.Append(wire)
                edges.Remove(1)

        faces = TopTools.TopTools_HSequenceOfShape()
        for i in range(closed_wires.Length()):
            face = BRepBuilderAPI.BRepBuilderAPI_MakeFace(
                closed_wires.Value(i + 1)
            ).Face()
            faces.Append(face)
        return (open_wires, closed_wires, faces, cross_section_bounding_box)

    @staticmethod
    def _draw(
        occ_display: OCCViewer.Viewer3d,
        objects: TopTools.TopTools_HSequenceOfShape,
        colour: Quantity.Quantity_Color,
    ) -> None:
        ## Render the objects onto the given display in the chosen colour
        #
        #  @param occ_display
        #    The viewer that the given objects should be rendered onto
        #  @param objects
        #    The different objects that should be drawn onto the given viewer
        #  @param colour
        #    The colour that should be used for drawing the corresponding objects

        for i in range(objects.Length()):
            ifcopenshell.geom.utils.display_shape(
                objects.Value(i + 1), colour, occ_display
            )
        return

    @staticmethod
    def _save_map(
        occ_display: OCCViewer.Viewer3d,
        export_dir: str,
        export_file_name: str,
        bounding_box: Bnd.Bnd_Box,
        resolution: float = 0.1,
    ) -> str:
        ## Save the map to a file as an image
        #
        # @param export_dir
        #   The directory where the map should be exported to
        # @param export_file_name
        #   The file name of the map to be exported (without or without the *.png suffix)
        # @param bounding_box
        #   The bounding box that should be considered for the plot
        # @param resolution
        #   Resolution of the map in [meters/pixel]
        # @return
        #   The complete path to the exported map file

        export_map_path = Path(os.path.join(export_dir, export_file_name)).with_suffix(
            ".png"
        )

        bounding_box_dim = bounding_box.Get()
        bounding_box_dx = bounding_box_dim[3] - bounding_box_dim[0]
        bounding_box_dy = bounding_box_dim[4] - bounding_box_dim[1]
        resolution_x = int(bounding_box_dx / resolution)
        resolution_y = int(bounding_box_dy / resolution)
        occ_display.SetSize(resolution_x, resolution_y)
        occ_display.View.TriedronErase()
        occ_display.View_Top()
        occ_display.View.FitAll(bounding_box)
        occ_display.View.Dump(str(export_map_path))
        # If IfcOpenShell is compiled without FreeImage the exported image might be shifted as described
        # in the following Github issue: https://github.com/tpaviot/pythonocc-core/issues/526

        # Remove blue-yellow aliasing effects by transforming to grayscale
        img = Image.open(export_map_path)
        grayscale_img = img.convert("L")
        binary_img = grayscale_img.point(
            lambda x: 0 if x < 72 else 255, "1"
        )  # Create binary image with lambda
        binary_img.save(export_map_path)

        return str(export_map_path)

    @staticmethod
    def _save_config_file(
        export_dir: str,
        export_file_name: str,
        map_path: str,
        resolution: float,
        origin: Tuple[float, float, float] = (0.0, 0.0, 0.0),
        occupied_threshold: float = 0.65,
        free_threshold: float = 0.196,
        is_negate: bool = False,
    ) -> str:
        ## Save the map configuration file required for the ROS map server
        # More information on the parameters can be found on http://wiki.ros.org/map_server#YAML_format
        #
        # @param export_dir
        #   The directory where the *.yaml map configuration file should be exported to
        # @param export_file_name
        #   The file name of the *.yaml map configuration file to be exported (without or without the
        #   *.yaml suffix)
        # @param map_path
        #   Path to the image file containing the occupancy data: Can be absolute, or relative to the location
        #   of the *.yaml file
        # @param resolution
        #   Resolution of the map in [meters/pixel]
        # @param origin
        #   The 2D pose of the lower-left pixel in the map, as (x, y, yaw), with yaw as counterclockwise
        #   rotation (yaw = 0 means no rotation)
        # @param occupied_threshold
        #   Pixels with occupancy probability greater than this threshold are considered completely occupied
        # @param free_threshold
        #   Pixels with occupancy probability less than this threshold are considered completely free
        # @param is_negate
        #   Whether the white/black free/occupied semantics should be reversed. The interpretation of
        #   the thresholds is unaffected!
        # @return
        #   The location of the exported *.yaml configuration file

        export_yaml_path = Path(os.path.join(export_dir, export_file_name)).with_suffix(
            ".yaml"
        )
        config_file = open(export_yaml_path, "w+")
        config_file.write("image: {}\n".format(map_path))
        config_file.write("resolution: {}\n".format(resolution))
        config_file.write(
            "origin: [ {0[0]:.3f}, {0[1]:.3f}, {0[2]:.3f} ]\n".format(origin)
        )
        config_file.write("occupied_thresh: {:.3f}\n".format(occupied_threshold))
        config_file.write("free_thresh: {:.3f}\n".format(free_threshold))
        config_file.write("negate: {}".format(int(is_negate)))
        config_file.close()
        return str(export_yaml_path)
