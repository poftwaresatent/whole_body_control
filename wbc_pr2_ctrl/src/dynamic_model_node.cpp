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
   \file dynamic_model_node.cpp Publishes partial joint-space dynamic models based on the /joint_states topic.
   \author Roland Philippsen
*/

#include <ros/ros.h>
#include <stanford_wbc_msgs/DynamicModel.h>
#include <jspace/tao_util.hpp>
#include <stanford_wbc/ros/Model.hpp>
#include <jspace/Model.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/shared_ptr.hpp>
#include <sensor_msgs/JointState.h>
#include <tao/dynamics/taoNode.h>

using namespace stanford_wbc_msgs;
using namespace boost;
using namespace std;

static jspace::State jspace_state;
static scoped_ptr<jspace::Model> jspace_model;
static vector<size_t> joint_index;
static ros::Subscriber state_sub;
static ros::Publisher model_pub;
static DynamicModel model_msg;
static size_t ndof;
static jspace::Vector gravity;
static jspace::Vector coriolis;
static jspace::Matrix mass_inertia;
static jspace::Matrix inv_mass_inertia;


static void state_cb(shared_ptr<sensor_msgs::JointState const> const & js);


int main(int argc, char*argv[])
{
  ros::init(argc, argv, "dynamic_model");
  ros::NodeHandle nn("~");
  
  jspace::ros::Model jspace_ros_model("/pr2_stanford_wbc/");
  
  try {
    
    ROS_INFO ("initializing model");
    
    static const size_t n_tao_trees(2);
    jspace_ros_model.initFromParam(nn, "/robot_description", n_tao_trees);
    jspace_model.reset(new jspace::Model());
    if (0 != jspace_model->init(jspace_ros_model.tao_trees_[0], jspace_ros_model.tao_trees_[1], &std::cerr)) {
      throw std::runtime_error("jspace_model->init() failed");
    }
    
    for (std::vector<std::string>::const_iterator gclink(jspace_ros_model.gravity_compensated_links_.begin());
	 gclink != jspace_ros_model.gravity_compensated_links_.end(); ++gclink) {
      taoDNode const * node(jspace_model->getNodeByName(*gclink));
      if ( ! node) {
	throw std::runtime_error("gravity-compensated link " + *gclink + " is not part of the jspace::Model");
      }
      int const id(node->getID());
      jspace_model->disableGravityCompensation(id, true);
      ROS_INFO ("disabled gravity compensation for link %s (ID %d)", gclink->c_str(), id);
    }
    
    ndof = jspace_model->getNDOF();
    jspace_state.init(ndof, ndof, 0);
    
    joint_index.clear(); // paranoid... has to be empty because of lazy init
    
    ROS_INFO ("setting up communication");
    
    state_sub = nn.subscribe("/joint_states", 1, state_cb); // should get this name from param server
    model_pub = nn.advertise<DynamicModel>("model", 1);
    
    model_msg.ok = false;
    model_msg.errstr = "not initialized";
    model_msg.root_name = jspace_ros_model.tao_root_name_;
    model_msg.name.resize(ndof);
    for (size_t ii(0); ii < ndof; ++ii) {
      model_msg.name[ii] = jspace_ros_model.tao_trees_[0]->info[ii].joint_name;
    }
    model_msg.position.resize(ndof);
    model_msg.velocity.resize(ndof);
    model_msg.gravity.resize(ndof);
    model_msg.coriolis_centrifugal.resize(ndof);
    model_msg.mass_inertia.resize(ndof * ndof);
    model_msg.inv_mass_inertia.resize(ndof * ndof);
    
  }
  catch (std::exception const & ee) {
    ROS_ERROR ("EXCEPTION %s", ee.what());
    exit(EXIT_FAILURE);
  }
  
  ROS_INFO ("spinning");
  
  ros::spin();
}


void state_cb(shared_ptr<sensor_msgs::JointState const> const & js)
{
  model_msg.header = js->header;
  
  if ((js->name.size() != js->position.size())
      || (js->name.size() != js->velocity.size())) {
    model_msg.ok = false;
    model_msg.errstr = "inconsistent sensor_msgs::JointState (sizes of name, position, and velocity must be identical)";
    model_pub.publish(model_msg);
    return;
  }
  
  if (joint_index.empty()) {
    // lazy init, cannot know joint index mapping before first
    // joint_state arrives
    joint_index.resize(ndof);
    for (size_t ii(0); ii < ndof; ++ii) {
      bool found(false);
      for (size_t jj(0); jj < js->name.size(); ++jj) {
	if (model_msg.name[ii] == js->name[jj]) {
	  found = true;
	  joint_index[ii] = jj;
	  break;
	}
      }
      if ( ! found) {
	joint_index.clear();	// try again next time
	model_msg.ok = false;
	model_msg.errstr = "joint '" + model_msg.name[ii] + "' not found in joint_state";
	model_pub.publish(model_msg);
	return;
      }
    }
    // could be paranoid and check for uniqueness of each joint_index...
  }
  
  // copy joint state over to the dynamics model
  for (size_t ii(0); ii < ndof; ++ii) {
    if (js->position.size() <= joint_index[ii]) {
      joint_index.clear();	// try again next time
      model_msg.ok = false;
      model_msg.errstr = "invalid index for joint '" + model_msg.name[ii] + "'";
      model_pub.publish(model_msg);
      return;
    }
    jspace_state.position_[ii] = js->position[joint_index[ii]];
    jspace_state.velocity_[ii] = js->velocity[joint_index[ii]];
  }
  jspace_model->update(jspace_state);
  
  // repackage the interesting chunks of information
  jspace_model->getGravity(gravity);
  jspace_model->getCoriolisCentrifugal(coriolis);
  jspace_model->getMassInertia(mass_inertia);
  jspace_model->getInverseMassInertia(inv_mass_inertia);
  for (size_t ii(0); ii < ndof; ++ii) {
    model_msg.position[ii] = jspace_state.position_[ii];
    model_msg.velocity[ii] = jspace_state.velocity_[ii];
    model_msg.gravity[ii] = gravity.coeff(ii);
    model_msg.coriolis_centrifugal[ii] = coriolis.coeff(ii);
    for (size_t jj(0); jj < ndof; ++jj) {
      model_msg.mass_inertia[ii + jj*ndof] = mass_inertia.coeff(ii, jj);
      model_msg.inv_mass_inertia[ii + jj*ndof] = inv_mass_inertia.coeff(ii, jj);
    }
  }
  
  // success, yeah
  model_msg.ok = true;
  model_msg.errstr = "";
  model_pub.publish(model_msg);
}
