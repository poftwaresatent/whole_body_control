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
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <err.h>
#include <errno.h>

using namespace wbc_m3_ctrl;
using namespace wbcnet;
using namespace std;

int main(int argc, char ** argv)
{
  try {
    
    if (argc != 4) {
      errx(EXIT_FAILURE, "usage: sendgoal x y z");
    }
    
    master_to_slave data;
    if (1 != sscanf(argv[1], "%lf", &(data.eepos_x))) {
      errx(EXIT_FAILURE, "failed to parse x");
    }
    if (1 != sscanf(argv[2], "%lf", &(data.eepos_y))) {
      errx(EXIT_FAILURE, "failed to parse y");
    }
    if (1 != sscanf(argv[3], "%lf", &(data.eepos_z))) {
      errx(EXIT_FAILURE, "failed to parse z");
    }
    
    int sockfd(create_udp_client("127.0.0.1", WBC_M3_CTRL_M2S_PORT, AF_UNSPEC));
    cout << "sending " << data.eepos_x << "  " << data.eepos_y << "  " << data.eepos_z << "\n";
    
    int const nwritten(udp_client_write(sockfd, &data, sizeof(data)));
    if (0 > nwritten) {
      err(EXIT_FAILURE, "write");
    }
    
  }
  
  catch (std::exception const & ee) {
    errx(EXIT_FAILURE, "EXCEPTION: %s", ee.what());
  }
  
}
