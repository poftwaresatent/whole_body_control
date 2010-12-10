/*
 * PR2 controller plugin for Stanford-WBC http://stanford-wbc.sourceforge.net/
 *
 * Copyright (c) 2010 Stanford University. All rights reserved.
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
   \file test_plugin.cpp
   \brief Plugin that directly implements a whole-body controller.
   \author Roland Philippsen
*/

#include <pr2_controller_interface/controller.h>
#include <pluginlib/class_list_macros.h>
#include <ros/console.h>


class TestPlugin
  : public pr2_controller_interface::Controller
{
public:
  TestPlugin();
  virtual ~TestPlugin();
  
  virtual void update(void);
  virtual bool init(pr2_mechanism_model::RobotState * robot, ros::NodeHandle &nn);
  
  int tick_;
};


TestPlugin::
TestPlugin()
  : tick_(0)
{
}


TestPlugin::
~TestPlugin()
{
}


void TestPlugin::
update(void)
{
  if (0 == (tick_ % 100)) {
    std::cerr << ".";
    if (0 == (tick_ % 5000)) {
      ROS_INFO ("test plugin still rockin!");
      std::cerr << "\n";
    }
  }
  ++tick_;
}


bool TestPlugin::
init(pr2_mechanism_model::RobotState * robot, ros::NodeHandle & nn)
{
  tick_ = 0;
  ROS_INFO ("test plugin ready to rock!");
  return true;
}


PLUGINLIB_REGISTER_CLASS (TestPlugin, TestPlugin, pr2_controller_interface::Controller)
