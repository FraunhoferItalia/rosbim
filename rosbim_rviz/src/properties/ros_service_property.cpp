// Copyright 2022-2024 Fraunhofer Italia Research

// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Copyright 2022 Fraunhofer Italia Research. All Rights Reserved.

#include "properties/ros_service_property.hpp"

#include <QtCore/qobjectdefs.h>

#include <QApplication>
#include <QCursor>
#include <QObject>
#include <QString>
#include <array>
#include <cstdio>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

namespace rosbim_rviz
{

/**\fn exec
   * \brief
   *   Execute a console command given by the command string
   *   We need to get the return value of the command line. Thus std::system is not sufficient
   *   Based on https://stackoverflow.com/a/478960 using popen() https://man7.org/linux/man-pages/man3/popen.3.html
   *
   * \warning
   *   This implementation only works on Linux as it uses popen()
   * 
   * \param[in] cmd
   *   The command to be executed inside the console
   * \return
   *   The output of the console after launching the command
  */
std::string exec(std::string const & cmd)
{
  std::array<char, 128> buffer{};
  std::string result{};
  std::unique_ptr<FILE, decltype(&pclose)> pipe{popen(cmd.c_str(), "r"), pclose};
  if (!pipe) {
    throw std::runtime_error("popen() failed!");
  }
  while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
    result += buffer.data();
  }
  return result;
}

RosServiceProperty::RosServiceProperty(
  QString const & name, QString const & default_value, QString const & service_type,
  QString const & description, rviz_common::properties::Property * parent,
  char const * changed_slot, QObject * receiver)
: EditableEnumProperty{name, default_value, description, parent, changed_slot, receiver},
  service_type_{service_type}
{
  connect(this, SIGNAL(requestOptions(EditableEnumProperty *)), this, SLOT(fillServiceList()));
  return;
}

void RosServiceProperty::fillServiceList()
{
  QApplication::setOverrideCursor(QCursor(Qt::WaitCursor));
  clearOptions();

  std::string const std_service_type = service_type_.toStdString();

  // Could not find a more convenient way through ROS Master API http://wiki.ros.org/ROS/Master_API
  // Also rosservice list performs this in several steps over XMLRPC and does not expose a single command
  // https://github.com/ros/ros_comm/blob/noetic-devel/tools/rosservice/src/rosservice/__init__.py
  std::istringstream services{exec("ros2 service find " + std_service_type)};
  for (std::string service; std::getline(services, service);) {
    addOptionStd(service);
  }
  sortOptions();
  QApplication::restoreOverrideCursor();
  return;
}

}  // namespace rosbim_rviz
