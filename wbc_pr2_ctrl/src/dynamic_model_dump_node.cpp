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
   \file dynamic_model_dump_node.cpp Listens to what dynamic_model_node publishes, and dumps it in a "pretty" way.
   \author Roland Philippsen
*/

#include <ros/ros.h>
#include <stanford_wbc_msgs/DynamicModel.h>
#include <boost/shared_ptr.hpp>
#include <stdio.h>

using namespace stanford_wbc_msgs;
using namespace boost;
using namespace std;

static ros::Subscriber model_sub;
static DynamicModel model_msg;


static void model_cb(shared_ptr<DynamicModel const> const & dm);


int main(int argc, char*argv[])
{
  ros::init(argc, argv, "dynamic_model_dump");
  ros::NodeHandle nn("~");
  model_sub = nn.subscribe("/dynamic_model/model", 1, model_cb); // should get this name from param server
  ros::spin();
}


static void pretty_number(char * buf, size_t buflen, double number)
{
  memset(buf, '\0', buflen);
  --buflen;
  if (isinf(number)) {
    snprintf(buf, buflen, "   inf  ");
  }
  else if (isnan(number)) {
    snprintf(buf, buflen, "   nan  ");
  }
  else if (fabs(fmod(number, 1)) < 1e-4) {
    snprintf(buf, buflen, " %- 7d", static_cast<int>(rint(number)));
  }
  else {
    snprintf(buf, buflen, "% 8.4f", number);
  }
}


void model_cb(shared_ptr<DynamicModel const> const & dm)
{
  printf("time: %zu.%09zu", (size_t) dm->header.stamp.sec, (size_t) dm->header.stamp.nsec);
  
  if ( ! dm->ok) {
    printf("\tERROR: %s\n", dm->errstr.c_str());
    return;
  }
  printf("\troot: %s\n", dm->root_name.c_str());
  
  size_t const ndof(dm->name.size());
  if ((ndof != dm->position.size())
      || (ndof != dm->velocity.size())
      || (ndof != dm->gravity.size())
      || (ndof != dm->coriolis_centrifugal.size())
      || (ndof * ndof != dm->mass_inertia.size())
      || (ndof * ndof != dm->inv_mass_inertia.size())) {
    printf("ERROR: inconsistent sizes in DynamicModel message\n");
    return;
  }
  
  printf("              ");
  for (size_t ii(0); ii < ndof; ++ii) {
    printf(" %8.8s", dm->name[ii].c_str());
  }
  
  static size_t const buflen(32);
  vector<char> cbuf(buflen);
  char * buf(&cbuf[0]);
  
  printf("\nposition     ");
  for (size_t ii(0); ii < ndof; ++ii) {
    pretty_number(buf, buflen, dm->position[ii]);
    printf(" %s", buf);
  }
  
  printf("\nvelocity     ");
  for (size_t ii(0); ii < ndof; ++ii) {
    pretty_number(buf, buflen, dm->velocity[ii]);
    printf(" %s", buf);
  }
  
  printf("\ngravity      ");
  for (size_t ii(0); ii < ndof; ++ii) {
    pretty_number(buf, buflen, dm->gravity[ii]);
    printf(" %s", buf);
  }
  
  printf("\nCoriolis-cor.");
  for (size_t ii(0); ii < ndof; ++ii) {
    pretty_number(buf, buflen, dm->coriolis_centrifugal[ii]);
    printf(" %s", buf);
  }
  
  for (size_t jj(0); jj < ndof; ++jj) {
    if (0 == jj) {
      printf("\nmass-inertia ");
    }
    else {
      printf("\n             ");
    }
    for (size_t ii(0); ii < ndof; ++ii) {
      pretty_number(buf, buflen, dm->mass_inertia[ii + jj*ndof]);
      printf(" %s", buf);
    }
  }
  
  for (size_t jj(0); jj < ndof; ++jj) {
    if (0 == jj) {
      printf("\ninv. mass-in.");
    }
    else {
      printf("\n             ");
    }
    for (size_t ii(0); ii < ndof; ++ii) {
      pretty_number(buf, buflen, dm->inv_mass_inertia[ii + jj*ndof]);
      printf(" %s", buf);
    }
  }
  
  printf("\n");
}
