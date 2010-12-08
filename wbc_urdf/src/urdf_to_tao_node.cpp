/*
 * Copyright (c) 2010 Stanford University and Willow Garage, Inc.
 *
 * This program is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this program.  If not, see
 * <http://www.gnu.org/licenses/>
 */

/**
   \file urdf_to_tao_node.cpp Converts the contents of the
   robot_description ROS parameter to a TAO tree and then dumps that
   in XML format to a temporary file.
   
   \author Roland Philippsen
*/

#include <ros/ros.h>
#include <jspace/tao_dump.hpp>
#include <wbc_urdf/Model.hpp>
#include <string.h>
#include <errno.h>
#include <stdlib.h>
#include <err.h>

using namespace std;

int main(int argc, char*argv[])
{
  ros::init(argc, argv, "urdf_to_tao");
  ros::NodeHandle nn("~");
  
  std::string format("SAI");
  if (argc > 1) {
    std::string arg(argv[1]);
    if (arg == "-l") {
      format = "Lotus";
    }
    else {
      errx(EXIT_FAILURE, "invalid argument `%s', use `-l' to select Lotus format instead of SAI", argv[1]);
    }
  }
  
  try {
    char tmpname[64];
    memset(tmpname, '\0', 64);
    strncpy(tmpname, "tao_tree.xml.XXXXXX", 63);
    int const tmpfd(mkstemp(tmpname));
    if (-1 == tmpfd) {
      throw runtime_error(string("mkstemp(") + tmpname + "): " + strerror(errno));
    }
    
    jspace::ros::Model jspace_ros_model("/pr2_stanford_wbc/");
    jspace_ros_model.initFromParam(nn, "/robot_description", 1);
    
    ostringstream xml_stream;
    if (format == "SAI") {
      dump_tao_tree_info_saixml(xml_stream, jspace_ros_model.tao_trees_[0]);
    }
    else {
      dump_tao_tree_info_lotusxml(xml_stream, "robot", "root", jspace_ros_model.tao_trees_[0]);
    }
    string const xml(xml_stream.str());
    size_t const len(xml.size());
    if (static_cast<ssize_t>(len) != write(tmpfd, xml.c_str(), len)) {
      throw runtime_error("write(): " + string(strerror(errno)));
    }
    close(tmpfd);
    
    cout << "wrote the TAO tree in " << format << "  format to " << tmpname << "\n";
  }
  catch (exception const & ee) {
    ROS_ERROR ("EXCEPTION %s", ee.what());
    exit(EXIT_FAILURE);
  }
  
  ros::shutdown();
  exit(EXIT_SUCCESS);
}
