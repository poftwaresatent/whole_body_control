ROS stack for Stanford Whole-Body Controller
==============================================

The integration of Stanford-WBC in ROS started in late 2009, as part
of a project supported by [Willow Garage][WG]. The aim of this project
is to perform whole-body control in [PR2][], and as a result one of
the main outcomes has been to open-source the core Stanford-WBC
codebase. We are currently finalizing a first release that enables
compliant control of operational tasks in the end effectors while
controlling compliant postures in the null-space of the task. The
medium term objective is to take into account motor torque limitations
while optimizing the execution of tasks and postures for the desired
behaviors. The long term objective is to provide an easy to reuse
framework for creating new compliant whole-body behavior by composing
existing ones and easily adding custom tasks.

[WG]: http://www.willowgarage.com/
[PR2]: http://www.willowgarage.com/pages/pr2/overview


Installation
------------

1. install [ROS](http://www.ros.org/wiki/ROS/Installation)
   (you will need PR2-specific stacks, including PR2 simulator)

2. clone the stack repos into your `ROS_PACKAGE_PATH`, e.g. in `~/ros/stacks`
   - cd ~/ros/stacks
   - git clone git://github.com/poftwaresatent/whole_body_control.git

3. use `git submodule` to pull in the core Stanford_WBC library code
   - cd whole_body_control
   - git submodule init
   - git submodule update

4. build it using `rosmake`
   - rosmake


Stack Contents
--------------

- `wbc_msgs` provides [message types][msg] and no additional code.  It
  has very few dependencies, and thus helps with keeping other
  packages decoupled from each other.

[msg]: http://www.ros.org/wiki/msg

- `wbc_core` is a bare ROS wrapper around the core stanford_wbc
  library. This core is kept entirely independent of ROS.

- `wbc_urdf` contains code for converting rigid body dynamic models
   from [URDF][] descriptions to the representation used by stanford_wbc,
   along with a few other utilities.

[URDF]: http://www.ros.org/wiki/urdf

- `wbc_pr2_ctrl` implements [pr2_controller_interface][plugin] plugins
   and related utilities to actually control a robot.  This package is
   where most of the "exciting" developments happen, and it contains
   the most end-user visible parts of the project.

[plugin]: http://www.ros.org/wiki/pr2_controller_interface

- `reflexxes_otg` is a library for online generation of
   acceleration-bounded trajectories, developed by [Reflexxes
   GmbH][reflexxes].  It is used for our current (December 2010)
   development efforts on handling motor torque limitations.  The idea
   is to limit task accelerations, such that the resulting joint
   torques stay within bounds.

[reflexxes]: http://www.reflexxes.net/


Run the PR2 Task / Nullspace-Posture Example in Gazebo
------------------------------------------------------

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

This example is implemented in the `wbc_pr2_ctrl/src/wbc_plugin.cpp`
file. Please note that this implementation is **not intended for
real-time** execution on the physical PR2: it produces very verbose
console output and uses a somewhat ad-hoc approach for integrating a
ROS service server into the plugin to allow modifying goals and gains
while the controller is running.

- terminal 1:
  - roscd wbc_pr2_ctrl/launch
  - roslaunch pr2_gazebo.launch

- terminal 2:
  - roscd wbc_pr2_ctrl/launch
  - roslaunch pr2_wbc_plugin.launch

- terminal 3:
  - roscd wbc_pr2_ctrl/scripts
  - ./task_posture_gui.py
  - Click "initialize"
  - Change some values
  - Click on the "Set FOO_BAR" buttons
  - See what happens, pause Gazebo to read the debug output.
