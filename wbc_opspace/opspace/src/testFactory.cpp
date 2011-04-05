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
#include <opspace/Skill.hpp>
#include <opspace/Factory.hpp>
#include <opspace/parse_yaml.hpp>
#include <stdexcept>

using namespace opspace;
using boost::shared_ptr;
using jspace::State;
using namespace std;


TEST (parse, tasks_only)
{
  static char * const yaml_string =
    "- tasks:\n"
    "  - type: opspace::SelectedJointPostureTask\n"
    "    name: odd\n"
    "    selection: [  1.0,  0.0,  1.0,  0.0,  1.0,  0.0 ]\n"
    "    kp: 100.0\n"
    "    kd:  20.0\n"
    "  - type: opspace::SelectedJointPostureTask\n"
    "    name: even\n"
    "    selection: [  0.0,  1.0,  0.0,  1.0,  0.0,  1.0 ]\n"
    "    kp: 100.0\n"
    "    kd:  20.0\n"
    "  - type: opspace::PositionTask\n"
    "    name: eepos\n"
    "    dt_seconds: 0.002\n"
    "    kp: [ 100.0 ]\n"
    "    kd: [  20.0 ]\n"
    "    maxvel: [ 0.5 ]\n"
    "    maxacc: [ 1.5 ]\n"
    "  - type: opspace::PostureTask\n"
    "    name: posture\n"
    "    dt_seconds: 0.002\n"
    "    kp: [ 400.0, 400.0, 400.0, 100.0, 100.0, 100.0, 100.0 ]\n"
    "    kd: [  40.0,  40.0,  40.0,  20.0,  20.0,  20.0,  20.0 ]\n"
    "    maxvel: [ 3.1416 ]\n"
    "    maxacc: [ 6.2832 ]\n";
  
  try {
    Factory factory(&cout);
    Status st;
    st = factory.parseString(yaml_string);
    EXPECT_TRUE (st.ok) << st.errstr;
    EXPECT_FALSE (factory.getTaskTable().empty()) << "task table should not be empty";
    EXPECT_TRUE (factory.getSkillTable().empty()) << "skill table should be empty";
    factory.dump(cout, "*** dump of factory", "* ");
  }
  catch (YAML::Exception const & ee) {
    ADD_FAILURE () << "unexpected YAML::Exception: " << ee.what();
  }
  catch (std::runtime_error const & ee) {
    ADD_FAILURE () << "unexpected std::runtime_error: " << ee.what();
  }
}


TEST (parse, tasks_and_skills)
{
  static char * const yaml_string =
    "- tasks:\n"
    "  - type: opspace::SelectedJointPostureTask\n"
    "    name: odd_instance\n"
    "    selection: [  1.0,  0.0,  1.0,  0.0,  1.0,  0.0 ]\n"
    "    kp: 100.0\n"
    "    kd:  20.0\n"
    "  - type: opspace::SelectedJointPostureTask\n"
    "    name: even_instance\n"
    "    selection: [  0.0,  1.0,  0.0,  1.0,  0.0,  1.0 ]\n"
    "    kp: 100.0\n"
    "    kd:  20.0\n"
    "  - type: opspace::PositionTask\n"
    "    name: eepos_instance\n"
    "    dt_seconds: 0.002\n"
    "    kp: [ 100.0 ]\n"
    "    kd: [  20.0 ]\n"
    "    maxvel: [ 0.5 ]\n"
    "    maxacc: [ 1.5 ]\n"
    "  - type: opspace::PostureTask\n"
    "    name: posture_instance\n"
    "    dt_seconds: 0.002\n"
    "    kp: [ 400.0, 400.0, 400.0, 100.0, 100.0, 100.0, 100.0 ]\n"
    "    kd: [  40.0,  40.0,  40.0,  20.0,  20.0,  20.0,  20.0 ]\n"
    "    maxvel: [ 3.1416 ]\n"
    "    maxacc: [ 6.2832 ]\n"
    "- skills:\n"
    "  - type: opspace::TPSkill\n"
    "    name: tpb\n"
    "    default:\n"
    "      eepos: eepos_instance\n"
    "      posture: posture_instance\n";
  
  try {
    Factory factory(&cout);
    Status st;
    st = factory.parseString(yaml_string);
    EXPECT_TRUE (st.ok) << st.errstr;
    EXPECT_FALSE (factory.getTaskTable().empty()) << "task table should not be empty";
    EXPECT_FALSE (factory.getSkillTable().empty()) << "skills table should not be empty";
    factory.dump(cout, "*** dump of factory", "* ");
  }
  catch (YAML::Exception const & ee) {
    ADD_FAILURE () << "unexpected YAML::Exception: " << ee.what();
  }
  catch (std::runtime_error const & ee) {
    ADD_FAILURE () << "unexpected std::runtime_error: " << ee.what();
  }
}


int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS ();
}
