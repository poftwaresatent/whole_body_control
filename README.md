Whole-Body Control ROS Stack
============================

The Whole-Body Control framework developed at the [Stanford Robotics
and AI Lab][manips] provides advanced control for human-centered
robotics and mobile manipulation.  This project provides a framework
for developing robot behaviors that use operational-space hierarchical
task decompositions, based on the work of many contributors over many
years, under the guidance of and in collaboration with [Oussama
Khatib][]: most notably [Jaeheung Park][], K. C. Chang, Diego Ruspini,
[Roy Featherstone][], Bob Holmberg, [Francois Conti][], [Roland
Philippsen][] and [Luis Sentis][].

The integration of Stanford-WBC as a PR2-specific plugin in ROS
started in late 2009, as part of a project at the [Stanford Robotics
and AI Lab][manips] which was supported by [Willow Garage][WG]. The
aim of this project was to perform whole-body control in [PR2][]: in
late 2010 we finalized a first release on PR2 that enables compliant
control of operational tasks in the end effectors while controlling
compliant postures in the null-space of the task. In early 2011 we
achieved first results with heuristics to take into account motor
torque limitations, integrate motion of the mobile base, and other
PR2-specific characteristics.

In the meantime, we have added support for the [Meka Robot][meka] at
the [Human-Centered Robotics Group (HCRL)][hcrl] of the University of
Texas at Austin. The long term objective of HCRL's developments within
ROS is to provide an easy to reuse framework for creating new
compliant whole-body behavior by composing existing ones and adding
new custom tasks, for any mobile manipulator that can be used with
torque control.

[manips]: http://cs.stanford.edu/groups/manips/
[Oussama Khatib]: http://cs.stanford.edu/groups/manips/people/oussama-khatib
[Jaeheung Park]: http://plaza4.snu.ac.kr/~park73/wiki/index.php5/People
[Roy Featherstone]: http://users.cecs.anu.edu.au/~roy/
[Francois Conti]: http://cs.stanford.edu/groups/manips/people/francois-conti
[Roland Philippsen]: http://cs.stanford.edu/groups/manips/people/roland-philippsen
[Luis Sentis]: http://www.me.utexas.edu/directory/faculty/sentis/luis/
[WG]: http://www.willowgarage.com/
[PR2]: http://www.willowgarage.com/pages/pr2/overview
[meka]: http://mekabot.com/
[hcrl]: http://www.me.utexas.edu/~hcrl/



Installation
============

1. install [ROS](http://www.ros.org/wiki/ROS/Installation)
   (you will need PR2-specific stacks, including PR2 simulator)

2. clone the stack repos into your `ROS_PACKAGE_PATH`, e.g. in `~/ros/stacks`
   - cd ~/ros/stacks
   - git clone git://github.com/poftwaresatent/whole_body_control.git

3. build it using `rosmake`
   - rosmake

Stack Contents
--------------

- `wbc_msgs` provides ROS [message][] and [service][] types.  It
  has very few dependencies, and thus helps with keeping other
  packages decoupled from each other.

[message]: http://www.ros.org/wiki/msg
[service]: http://www.ros.org/wiki/srv

- `wbc_core` is a ROS wrapper around the core [stanford_wbc][]
  library. This core is based on earlier work at Stanford University,
  most notably the TAO dynamics engine, and is kept entirely
  independent of ROS.

[stanford_wbc]: https://github.com/poftwaresatent/stanford_wbc

- `wbc_uta_opspace` is a ROS wrapper around the operational space
  extensions, developed at UT Austin, to the core Stanford_WBC
  operational space library. It is taken from the ROS-independent code
  base in the [UT Austin Whole-Body Control project][utaustin-wbc].

[utaustin-wbc]: https://github.com/poftwaresatent/utaustin-wbc

- `wbc_urdf` contains code for converting rigid body dynamic models
  from [URDF][] descriptions to the representation used by
  stanford_wbc, along with a few other utilities.

[URDF]: http://www.ros.org/wiki/urdf

- `wbc_pr2_ctrl` implements [pr2_controller_interface][plugin] plugins
   and related utilities to actually control PR2 (or any other robot
   that uses the pr2_controller_interface approach).

[plugin]: http://www.ros.org/wiki/pr2_controller_interface

- `wbc_m3_ctrl` uses the torque-shared-memory mode provided by Meka to
  control their M3 arm. It implements an [RTAI][] executable and ROS
  bindings in the form of messages and service.

[RTAI]: http://www.rtai.org/



Run the PR2 Task / Nullspace-Posture Example in Gazebo
======================================================

After successfully installing ROS and building the
`whole_body_control` stack, in particular the `wbc_pr2_ctrl` package,
you can run the following example of whole-body operational space
control. It is a "minimal" example in the sense that it contains two
tasks (a Cartesian end-effector position and a joint-space posture)
and that their interaction is hardcoded (you cannot easily change the
task hierarchy at runtime). Nevertheless, this is a complete example
of **dynamically consistent nullspace projection** techniques, which are
the foundations for the power and expressiveness of the whole-body
controller.

This example is implemented in the
`wbc_pr2_ctrl/src/opspace_servo.cpp` file. You can also use that as-is
for real-time execution on the physical PR2. The `opspace_servo` uses
the freshly designed and implemented **runtime configuration and
parameter reflection** capabilities of the UT Austin opspace library,
which allows changing the tasks and skills at startup time using a
YAML file, and lets us modify goals, gains, and other parameters while
the controller is running. These are all quite nifty features which we
are excited to share with the community.

**1. Launch PR2 in Gazebo**

    roscd wbc_pr2_ctrl/launch
    roslaunch pr2_gazebo.launch

**2. Launch the WBC pump plugin**

    roscd wbc_pr2_ctrl/launch
    roslaunch pr2_pump_plugin.launch

**3. Launch the WBC opspace controller**

    roscd wbc_pr2_ctrl/launch
    roslaunch pr2_opspace.launch

**4. List services and messages**

    rostopic list | fgrep opspace
    rosservice list | fgrep opspace

**5. Get and set end-effector trajectory goal using service**

    rosservice call /opspace_servo/get_param '{ com_type: task, com_name: eepos, param_name: trjgoal }'
    rosservice call /opspace_servo/set_param '{ com_type: task, com_name: eepos, param: { name: trjgoal, type: 4, realval: [0.6, 0.1, 1.0] } }'
    rosservice call /opspace_servo/set_param '{ com_type: task, com_name: eepos, param: { name: trjgoal, type: 4, realval: [0.7, -0.1, 0.8] } }'

**6. Set end-effector trajectory goal using message (param channel)**

    rosservice call /opspace_servo/open_channel '{ com_type: task, com_name: eepos, param_name: trjgoal }'
    (note the channel_id in the reply, it will probably be 0)
    rostopic pub -1 /opspace_servo/vector_channel wbc_msgs/VectorChannel '{ channel_id: 0, value: [0.5, -0.1, 0.8] }'
    rostopic pub -1 /opspace_servo/vector_channel wbc_msgs/VectorChannel '{ channel_id: 0, value: [0.5, 0.1, 0.8] }'
