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

#ifndef WBCNET_UDP_UTIL_HPP
#define WBCNET_UDP_UTIL_HPP

#include <iosfwd>
#include <stdexcept>

extern "C" {
#include <sys/types.h>
#include <sys/socket.h>
}

namespace wbcnet {
  
  int create_udp_server(/** port specification, will get passed to getaddrinfo() */
			char const * port,
			/** use AF_UNSPEC for allowing IPv4 or IPv6,
			    AF_INET for IPv4, or AF_INET6 for IPv6 */
			int ai_family) throw(std::runtime_error);
  
  int udp_server_recvfrom(int sfd, void * buf, size_t buf_len, int flags,
			  struct sockaddr * addr, socklen_t * addr_len);
  
  int udp_server_sendto(int sfd, void const * buf, size_t buf_len, int flags,
			struct sockaddr const * addr, socklen_t addr_len);
  
  int create_udp_client(/** Host to connect to. Gets passed to getaddrinfo(). */
			char const * host,
			/** Port to connect to. Gets passed to getaddrinfo(). */
			char const * port,
			/** use AF_UNSPEC for allowing IPv4 or IPv6,
			    AF_INET for IPv4, or AF_INET6 for IPv6 */
			int ai_family) throw(std::runtime_error);

  int udp_client_write(int cfd, void const * buf, size_t buf_len);
  
  int udp_client_read(int cfd, void * buf, size_t buf_len);
  
#ifdef LINUX
  int udp_disable_mtu(int fd);
  int udp_max_priority(int fd);
#endif /* LINUX */

  int udp_tos_lowdelay(int fd);
  
}

#endif // WBCNET_UDP_UTIL_HPP
