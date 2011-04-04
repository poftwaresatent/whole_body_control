/*
 * Whole-Body Control for Human-Centered Robotics http://www.me.utexas.edu/~hcrl/
 *
 * Copyright (c) 2011 University of Texas at Austin. All rights reserved.
 *
 * Author: Roland Philippsen
 *
 * BSD license:
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of
 *    contributors to this software may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHORS AND CONTRIBUTORS ``AS IS''
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR THE CONTRIBUTORS TO THIS SOFTWARE BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef WBC_OPSPACE_UTIL_H
#define WBC_OPSPACE_UTIL_H

#include <opspace/Factory.hpp>
#include <opspace/controller_library.hpp>
#include <wbc_msgs/SetParameter.h>
#include <wbc_msgs/GetParameter.h>
#include <wbc_msgs/ListParameters.h>
#include <wbc_msgs/OpenChannel.h>
#include <wbc_msgs/StringChannel.h>
#include <boost/shared_ptr.hpp>
#include <ros/ros.h>

namespace wbc_opspace {

  class ParamCallbacks
  {
  public:
    ParamCallbacks();
    
    void init(ros::NodeHandle node,
	      boost::shared_ptr<opspace::Factory> factory,
	      boost::shared_ptr<opspace::ControllerNG> controller,
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

    boost::shared_ptr<opspace::Factory> factory_;
    boost::shared_ptr<opspace::ControllerNG> controller_;
    
    std::map<int, opspace::StringParameter *> strings_;
    std::map<int, opspace::IntegerParameter *> integers_;
    std::map<int, opspace::RealParameter *> reals_;
    std::map<int, opspace::VectorParameter *> vectors_;
    std::map<int, opspace::MatrixParameter *> matrices_;
    
    int next_channel_id_;
  };
  
}

#endif // WBC_OPSPACE_UTIL_H
