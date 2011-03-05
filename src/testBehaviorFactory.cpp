/*
 * Whole-Body Control for Human-Centered Robotics http://www.me.utexas.edu/~hcrl/
 *
 * Copyright (c) 2011 University of Texas at Austin. All rights reserved.
 *
 * Authors: Roland Philippsen and Luis Sentis
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

#include <gtest/gtest.h>
#include <opspace/Behavior.hpp>
#include <opspace/Factory.hpp>
#include <opspace/parse_yaml.hpp>
#include <jspace/test/model_library.hpp>

using namespace opspace;
using boost::shared_ptr;
using jspace::State;
using namespace std;


static Model * get_puma()
{
  static Model * puma(0);
  if ( ! puma) {
    puma = jspace::test::create_puma_model();
  }
  size_t const ndof(puma->getNDOF());
  State state(ndof, ndof, 0);
  for (size_t ii(0); ii < ndof; ++ii) {
    state.position_[ii] = 0.01 * ii + 0.08;
    state.velocity_[ii] = 0.02 - 0.005 * ii;
  }
  puma->update(state);
  return puma;
}


TEST (behavior, parse)
{
  static char * const task_yaml =
    "- type: opspace::SelectedJointPostureTask\n"
    "  name: odd_instance\n"
    "  selection: [  1.0,  0.0,  1.0,  0.0,  1.0,  0.0 ]\n"
    "  kp: 100.0\n"
    "  kd:  20.0\n"
    "- type: opspace::SelectedJointPostureTask\n"
    "  name: even_instance\n"
    "  selection: [  0.0,  1.0,  0.0,  1.0,  0.0,  1.0 ]\n"
    "  kp: 100.0\n"
    "  kd:  20.0\n"
    "- type: opspace::PositionTask\n"
    "  name: eepos_instance\n"
    "  dt_seconds: 0.002\n"
    "  kp: [ 100.0 ]\n"
    "  kd: [  20.0 ]\n"
    "  maxvel: [ 0.5 ]\n"
    "  maxacc: [ 1.5 ]\n"
    "- type: opspace::PostureTask\n"
    "  name: posture_instance\n"
    "  dt_seconds: 0.002\n"
    "  kp: [ 400.0, 400.0, 400.0, 100.0, 100.0, 100.0, 100.0 ]\n"
    "  kd: [  40.0,  40.0,  40.0,  20.0,  20.0,  20.0,  20.0 ]\n"
    "  maxvel: [ 3.1416 ]\n"
    "  maxacc: [ 6.2832 ]\n";
  
  static char * const behavior_yaml =
    "- type: opspace::TPBehavior\n"
    "  name: tpb\n"
    "  default:\n"
    "    eepos: eepos_instance\n"
    "    posture: posture_instance\n";
  
  Model * puma(get_puma());

  Factory factory(&cout);
  Status st;
  st = factory.parseString(task_yaml);
  EXPECT_TRUE (st.ok) << st.errstr;
  
  std::istringstream behavior_is(behavior_yaml);
  try {
    YAML::Parser parser(behavior_is);
    YAML::Node doc;
    BehaviorParser bp(factory, &cout);
    
    while (parser.GetNextDocument(doc)) {
      for (size_t ii(0); ii < doc.size(); ++ii) {
	YAML::Node const & node(doc[ii]);
	node >> bp;
	if ( ! bp.behavior) {
	  throw std::runtime_error("oops, behavior_parser_s::behavior is zero");
	}
	delete bp.behavior;
      }
    }
  }
  
  catch (YAML::Exception const & ee) {
    ADD_FAILURE () << "YAML::Exception: " << ee.what();
  }
  catch (std::runtime_error const & ee) {
    ADD_FAILURE () << "std::runtime_error: " << ee.what();
  }
}


int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS ();
}
