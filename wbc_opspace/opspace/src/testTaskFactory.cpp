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

#include <opspace/TaskFactory.hpp>
#include <err.h>

using namespace jspace;
using namespace opspace;
using namespace std;

static char * const yaml_string =
  "- type: opspace::SelectedJointPostureTask\n"
  "  name: odd\n"
  "  selection: [  1.0,  0.0,  1.0,  0.0,  1.0,  0.0 ]\n"
  "  kp: 100.0\n"
  "  kd:  20.0\n"
  "- type: opspace::SelectedJointPostureTask\n"
  "  name: even\n"
  "  selection: [  0.0,  1.0,  0.0,  1.0,  0.0,  1.0 ]\n"
  "  kp: 100.0\n"
  "  kd:  20.0\n"
  "- type: opspace::PositionTask\n"
  "  name: eepos\n"
  "  dt_seconds: 0.002\n"
  "  kp: [ 100.0 ]\n"
  "  kd: [  20.0 ]\n"
  "  maxvel: [ 0.5 ]\n"
  "  maxacc: [ 1.5 ]\n"
  "- type: opspace::PostureTask\n"
  "  name: posture\n"
  "  dt_seconds: 0.002\n"
  "  kp: [ 400.0, 400.0, 400.0, 100.0, 100.0, 100.0, 100.0 ]\n"
  "  kd: [  40.0,  40.0,  40.0,  20.0,  20.0,  20.0,  20.0 ]\n"
  "  maxvel: [ 3.1416 ]\n"
  "  maxacc: [ 6.2832 ]\n";
  
int main(int argc, char ** argv)
{
  Status st;  
  TaskFactory tfac(&cout);
  
  if (argc > 1) {
    cout << "parsing file `" << argv[1] << "'\n";
    st = tfac.parseFile(argv[1]);
  }
  else {
    cout << "parsing yaml_string:\n" << yaml_string;
    st = tfac.parseString(yaml_string);
  }
  if ( ! st) {
    cout << "oops: " << st.errstr << "\n";
  }
  else {
    cout << "success\n";
  }
}
