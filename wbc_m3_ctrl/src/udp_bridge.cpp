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

#include <wbc_m3_ctrl/udp_util.h>
#include <wbc_m3_ctrl/qh.h>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <err.h>
#include <errno.h>

using namespace wbc_m3_ctrl;
using namespace wbcnet;
using namespace std;

static ros::Subscriber sub;
static ros::Publisher pub;
static m2s_data m2s;
static s2m_data s2m;
static int m2s_fd;
static int s2m_fd;

static void cb(std_msgs::Float64MultiArray const & msg_in)
{
  if (msg_in.data.size() < 3) {
    cerr << "\nE";
    return;
  }
  cerr << "I";
  m2s.eepos_x = 1e-3 * msg_in.data[0];
  m2s.eepos_y = 1e-3 * msg_in.data[1];
  m2s.eepos_z = 1e-3 * msg_in.data[2];
  if (0 > udp_client_write(m2s_fd, &m2s, sizeof(m2s))) {
    warn("\nudp_client_write");
  }
  else {
    cerr << "o";
  }
}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "wbc_m3_ctrl_udp_bridge");
  ros::NodeHandle node("~");
  
  try {
    m2s_fd = create_udp_client("127.0.0.1", WBC_M3_CTRL_M2S_PORT, AF_UNSPEC);
    s2m_fd = create_udp_server(WBC_M3_CTRL_S2M_PORT, AF_UNSPEC);
  }
  catch (std::runtime_error const & ee) {
    errx(EXIT_FAILURE, "failed to start servo: %s", ee.what());
  }
  
  sub = node.subscribe("/cart_pos_blacky", 1, cb);
  pub = node.advertise<std_msgs::Float64MultiArray>("/cart_pos_dreamer", 1);
  
  ros::Time t0(ros::Time::now());
  ros::Duration pub_dt(1e-3);
  std_msgs::Float64MultiArray msg_out;
  msg_out.data.assign(15, 0.0);
  struct sockaddr_storage peer_addr;
  socklen_t peer_addr_len;
  bool got_data(false);
  
  while (ros::ok()) {
    peer_addr_len = sizeof(struct sockaddr_storage);
    int const nread(udp_server_recvfrom(s2m_fd, &s2m, sizeof(s2m), MSG_DONTWAIT,
					(struct sockaddr *) &peer_addr, &peer_addr_len));
    if (0 > nread) {
      if ((EAGAIN != errno) && (EWOULDBLOCK != errno)) {
	warn("udp_server_recvfrom");
	ros::shutdown();
      }
    }
    else if (sizeof(s2m) == nread) {
      got_data = true;
      cerr << "i";
    }
    
    if (got_data) {
      ros::Time t1(ros::Time::now());
      if (t1 - t0 > pub_dt) {
	t0 = t1;
	msg_out.data[0] = 1e3 * s2m.eepos_x;
	msg_out.data[1] = 1e3 * s2m.eepos_y;
	msg_out.data[2] = 1e3 * s2m.eepos_z;
	pub.publish(msg_out);
	cerr << "O";
      }
    }
    
    ros::spinOnce();
  }
  
  cerr << "\nbyebye\n";
}
