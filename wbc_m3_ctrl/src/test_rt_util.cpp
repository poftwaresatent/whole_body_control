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

#include <wbc_m3_ctrl/rt_util.h>
#include <err.h>
#include <stdio.h>

namespace {
  
  class Test : public wbc_m3_ctrl::RTUtil {
  public:
    bool initialized, slowmedown;
    
    Test(): initialized(false), slowmedown(false) {}
    
    virtual int init(jspace::State const & state) {
      if (initialized) {
	fprintf(stderr, "ERROR in init(): already initialized\n");
	return 42;
      }
      initialized = true;
      fprintf(stderr, "hello from init()\n");
      return 0;
    }
    
    virtual int update(jspace::State const & state,
		       jspace::Vector & command)
    {
      if ( ! initialized) {
	fprintf(stderr, "ERROR in update(): not initialized\n");
	return 17;
      }
      command = -10.0 * state.velocity_;
      fprintf(stderr, "+");
      if (slowmedown) {
	fprintf(stderr, "update(): slow down requested...\n");
	usleep(5000);
	slowmedown = false;
      }
      return 0;
    }
    
    virtual int cleanup(void)
    {
      if ( ! initialized) {
	fprintf(stderr, "ERROR in cleanup(): not initialized\n");
	return 8876;
      }
      fprintf(stderr, "hello from cleanup()\n");
      initialized = false;
      return 0;
    }
    
    virtual int slowdown(long long iteration,
			 long long desired_ns,
			 long long actual_ns)
    {
      if ( ! initialized) {
	fprintf(stderr, "ERROR in slowdown(): not initialized\n");
	return 3232;
      }
      fprintf(stderr, "hello from slowdown()\n");
      return 0;
    }
    
  };
  
}


int main(int argc, char ** argv)
{
  Test test;
  try {
    test.start(500);
    for (size_t ii(0); ii < 10; ++ii) {
      fprintf(stderr, "\n*%zu*\n", ii);
      usleep(100000);
    }
    test.slowmedown = true;
    for (size_t ii(0); ii < 10; ++ii) {
      fprintf(stderr, "\n*%zu*\n", ii);
      usleep(100000);
    }
    test.shutdown();
  }
  catch (std::runtime_error const & ee) {
    errx(EXIT_FAILURE, "EXCEPTION: %s", ee.what());
  }
  fprintf(stderr, "Bye bye!\n");
}
