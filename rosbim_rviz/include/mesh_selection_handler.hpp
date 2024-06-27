/**
 * \file mesh_selection_handler.hpp
 * \mainpage
 *   Selection handler for the Mesh class: Can be used to interact with the mesh
 *   Largely based on https://github.com/ros-visualization/rviz/blob/noetic-devel/src/rviz/default_plugin/markers/marker_selection_handler.cpp
 * 
*/

#ifndef ROSBIM_RVIZ_MESH_SELECTION_HANDLER
#define ROSBIM_RVIZ_MESH_SELECTION_HANDLER

#include <OgreQuaternion.h>
#include <OgreVector3.h>

#include <QColor>
#include <rviz_common/display_context.hpp>
#include <rviz_common/interaction/forwards.hpp>
#include <rviz_common/interaction/selection_handler.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/property.hpp>
#include <rviz_common/properties/quaternion_property.hpp>
#include <rviz_common/properties/vector_property.hpp>
#include <string>

#include "mesh.hpp"

namespace rosbim_rviz
{

class Mesh;  // Forward-declaration of Mesh required for this header to compile

/**\class MeshSelectionHandler
   * \brief 
   *   Selection handler that allows the selection of a mesh and permits the user to visualize certain 
   *   information inside the Select panel
  */
class MeshSelectionHandler : public rviz_common::interaction::SelectionHandler
{
public:
  /**\fn MeshSelectionHandler
       * \brief
       *   The constructor
       *
       * \param[in] mesh
       *   The mesh pointer that the selection handler should act on
       * \param[in] name
       *   The name of the mesh
       * \param[in] context
       *   The context that the display is working in
      */
  MeshSelectionHandler(
    Mesh * mesh, std::string const & name, rviz_common::DisplayContext * context);

  /**\fn ~MeshSelectionHandler
       * \brief
       *   The destructor
      */
  ~MeshSelectionHandler() override;

  /**\fn createProperties
       * \brief
       *   Function for creating the different properties to be displayed inside RViz
       *   See also http://docs.ros.org/en/noetic/api/rviz/html/c++/classrviz_1_1SelectionHandler.html#ad13ce2cdcf1639a17a6aa6604809a847
       * 
       * \param[in] obj
       *   The picked object
       * \param[in] parent_property
       *   The property that the other properties should be inserted into
      */
  void createProperties(
    rviz_common::interaction::Picked const & obj,
    rviz_common::properties::Property * parent_property) override;

  /**\fn updateProperties
       * \brief
       *   Function for updating the different properties to be displayed
       *   See http://docs.ros.org/en/noetic/api/rviz/html/c++/classrviz_1_1SelectionHandler.html#a4f7c2e95a0c4d50a33ca2724eb1f0bab
      */
  void updateProperties() override;

protected:
  // The following functions are only used internally for updating the properties
  /**\fn getPosition
       * \brief
       *   Getter for the current position of the mesh
       *   Can't be constant due to the non-constant virtual function of rviz::Object
       *
       * \return
       *   The current position of the mesh with respect to its parent frame
      */
  Ogre::Vector3 getPosition();

  /**\fn getOrientation
       * \brief
       *   Getter for the current orientation of the mesh
       *   Can't be constant due to the non-constant virtual function of rviz::Object
       *
       * \return
       *   The current orientation of the mesh with respect to its parent frame
      */
  Ogre::Quaternion getOrientation();

  /**\fn getScale
       * \brief
       *   Getter for the current scale of the mesh
       *
       * \return
       *   The scale of the mesh for each individual axis
      */
  Ogre::Vector3 getScale() const;

  /**\fn getColor
       * \brief
       *   Getter for the current color of the mesh
       *
       * \return
       *   The current color of the mesh
      */
  QColor getColor() const;

  Mesh * mesh_;
  std::string name_;
  rviz_common::properties::VectorProperty * position_property_;
  rviz_common::properties::QuaternionProperty * orientation_property_;
  rviz_common::properties::VectorProperty * scale_property_;
  rviz_common::properties::ColorProperty * color_property_;
};

}  // namespace rosbim_rviz

#endif
