/*
 * Copyright (C) 2011 The Board of Trustees of The Leland Stanford Junior University. All rights reserved.
 *
 * Author: Roland Philippsen
 *         http://cs.stanford.edu/group/manips/
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

#ifndef WBC_CORE_OPSPACE_PARAM_CALLBACKS_HPP
#define WBC_CORE_OPSPACE_PARAM_CALLBACKS_HPP

#include <opspace/Parameter.hpp>
#include <wbc_msgs/SetParameter.h>
#include <wbc_msgs/GetParameter.h>
#include <wbc_msgs/ListParameters.h>
#include <wbc_msgs/OpenChannel.h>
#include <wbc_msgs/StringChannel.h>
#include <boost/shared_ptr.hpp>
#include <ros/ros.h>

namespace wbc_core_opspace {

  class ParamCallbacks
  {
  public:
    ParamCallbacks();
    
    void init(ros::NodeHandle node,
	      boost::shared_ptr<opspace::ReflectionRegistry> registry,
	      size_t input_queue_size,
	      size_t output_queue_size)
      throw(std::runtime_error);
    
    opspace::Parameter * findParam(std::string const & com_type,
				   std::string const & com_name,
				   std::string const & param_name,
				   std::string & errstr);
    
    bool setParam(wbc_msgs::SetParameter::Request & request,
		  wbc_msgs::SetParameter::Response & response);
    
    bool getParam(wbc_msgs::GetParameter::Request & request,
		  wbc_msgs::GetParameter::Response & response);
    
    bool listParams(wbc_msgs::ListParameters::Request & request,
		    wbc_msgs::ListParameters::Response & response);
    
    bool openChannel(wbc_msgs::OpenChannel::Request & request,
		     wbc_msgs::OpenChannel::Response & response);
    
    void stringChannel(wbc_msgs::StringChannel const & msg);
    
    void integerChannel(wbc_msgs::IntegerChannel const & msg);
    
    void realChannel(wbc_msgs::RealChannel const & msg);
    
    void vectorChannel(wbc_msgs::VectorChannel const & msg);
    
    void matrixChannel(wbc_msgs::MatrixChannel const & msg);
    
  protected:
    ros::ServiceServer set_param_;
    ros::ServiceServer get_param_;
    ros::ServiceServer list_params_;
    ros::ServiceServer open_channel_;
    ros::Subscriber string_sub_;
    ros::Subscriber integer_sub_;
    ros::Subscriber real_sub_;
    ros::Subscriber vector_sub_;
    ros::Subscriber matrix_sub_;
    ros::Publisher channel_feedback_;
    
    boost::shared_ptr<opspace::ReflectionRegistry> registry_;
    
    std::map<int, opspace::StringParameter *> strings_;
    std::map<int, opspace::IntegerParameter *> integers_;
    std::map<int, opspace::RealParameter *> reals_;
    std::map<int, opspace::VectorParameter *> vectors_;
    std::map<int, opspace::MatrixParameter *> matrices_;
    
    int next_channel_id_;
  };
  
}

#endif // WBC_CORE_OPSPACE_PARAM_CALLBACKS_HPP
