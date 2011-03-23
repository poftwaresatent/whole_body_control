/*
 * Copyright (c) 2008 Roland Philippsen <roland DOT philippsen AT gmx DOT net>
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
#include <iostream>
#include <stdlib.h>
#include <unistd.h>
#include <err.h>
#include <errno.h>
#include <fcntl.h>

using namespace wbcnet;
using namespace std;

struct ds {
  float jpos[7];
  float jvel[7];
};

static ds data;

int main(int argc, char ** argv)
{
  try {
    
    if (argc < 3) {
      errx(EXIT_FAILURE, "usage: test [s port | c host port]");
    }
    
    if (argv[1][0] == 's') {
      static char const * port = argv[2];    
      
      int sockfd(create_udp_server(port, AF_UNSPEC));
      if (0 > fcntl(sockfd, F_SETFL, O_NONBLOCK)) {
	err(EXIT_FAILURE, "fcntl");
      }
      while (true) {
	cerr << "receiving";
	ssize_t nread(0);
	struct sockaddr_storage peer_addr;
	socklen_t peer_addr_len(sizeof(struct sockaddr_storage));
	while (true) {
	  nread = udp_server_recvfrom(sockfd, &data, sizeof(data), MSG_DONTWAIT,
				      (struct sockaddr *) &peer_addr, &peer_addr_len);
	  if (0 < nread) {
	    break;
	  }
	  if (0 > nread) {
	    if ((EAGAIN == errno) || (EWOULDBLOCK == errno)) {
	      continue;
	    }
	    err(EXIT_FAILURE, "recvfrom");
	  }
	  cerr << ".";
	  usleep(300000);
	}
	if (sizeof(data) != nread) {
	  errx(EXIT_FAILURE, "received %d bytes but wanted %d", nread, sizeof(data));
	}
	cerr << "\n";
	for (size_t ii(0); ii < 7; ++ii) {
	  cerr << "  " << data.jpos[ii] << "  " << data.jvel[ii] << "\n";
	}
      }
      
    }
    
    else if (argv[1][0] == 'c') {
      if (argc < 4) {
	errx(EXIT_FAILURE, "usage: test [s port | c host port]");
      }
      static char const * host = argv[2];    
      static char const * port = argv[3];    
      
      int sockfd(create_udp_client(host, port, AF_UNSPEC));
      for (float foo(0); true; foo += 1.0) {
	cerr << "sending\n";
	for (size_t ii(0); ii < 7; ++ii) {
	  data.jpos[ii] = foo + 0.1 * ii;
	  data.jvel[ii] = foo - 0.1 * ii;
	  cerr << "  " << data.jpos[ii] << "  " << data.jvel[ii] << "\n";
	}
	int const nwritten(udp_client_write(sockfd, &data, sizeof(data)));
	if (0 > nwritten) {
	  err(EXIT_FAILURE, "write");
	}
	usleep(1000000);
      }
      
    }
    
    else {
      errx(EXIT_FAILURE, "usage: test [s port | c host port]");
    }
    
  }
  
  catch (std::exception const & ee) {
    errx(EXIT_FAILURE, "EXCEPTION: %s", ee.what());
  }
  
}
