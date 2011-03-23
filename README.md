Whole-Body Control ROS Stack
============================

The integration of Stanford-WBC in ROS started in late 2009, as part
of a project at the [Stanford Robotics and AI Lab][manips] which was
supported by [Willow Garage][WG]. The aim of this project was to
perform whole-body control in [PR2][], and as a result one of the main
outcomes has been to open-source the core Stanford-WBC codebase. In
the meantime, we have added support for the [Meka Robot][meka] at the
[Human-Centered Robotics Group][hcrl] of the University of Texas at
Austin. The long term objective is to provide an easy to reuse
framework for creating new compliant whole-body behavior by composing
existing ones and adding new custom tasks, for any mobile manipulator
that can be used with torque control.

In late 2010 we finalized a first release on PR2 that enables
compliant control of operational tasks in the end effectors while
controlling compliant postures in the null-space of the task. In early
2011 we achieved first results with heuristics to take into account
motor torque limitations, integrate motion of the mobile base, and
other PR2-specific characteristics.

[manips]: http://cs.stanford.edu/groups/manips/
[WG]: http://www.willowgarage.com/
[PR2]: http://www.willowgarage.com/pages/pr2/overview
[meka]: http://mekabot.com/
[hcrl]: http://www.me.utexas.edu/~hcrl/

Contributors
------------

The driving forces behind these developments are Luis Sentis for the
theoretical foundations and Roland Philippsen for the software. The
wealth of experience of Oussama Khatib has repeatedly fed into this
project, especially during its early stages, and Josh Petersen worked
on many PR2-specific experiments. The project relies on many
third-party sources and we are grateful for all their contributors who
produce the high-quality open-source code that allows us to focus on
our core goals.



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

- `wbc_opspace` is a ROS wrapper around the core
  [utaustin_wbc_opspace][] library, which provides operational space
  utilities that build on top of stanford_wbc and is likewise kept
  ROS-independent. Inspired by preliminary work done at Stanford, it
  has been completely redesigned and rewritten from scratch at the
  University of Texas at Austin.

[utaustin_wbc_opspace]: https://github.com/poftwaresatent/utaustin_wbc_opspace

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

- `reflexxes_otg` is a library for online generation of
   acceleration-bounded trajectories, developed by [Reflexxes
   GmbH][reflexxes].  It is used for our current (December 2010)
   development efforts on handling motor torque limitations.  The idea
   is to limit task accelerations, such that the resulting joint
   torques stay within bounds.

[reflexxes]: http://www.reflexxes.net/

Some Notes About git-subtree
----------------------------

We've stopped using `git-submodule` for the `wbc_core/src` directory
and have replaced it with an in-tree copy via [git-subtree][]. This
should make it possible to merge changes across projects that use the
upstream [Stanford WBC][stanford-wbc] codebase. Similarly, the
`wbc_opspace/opspace` directory is an in-tree copy of [UT Austin
WBC][utaustin-wbc].

Here are the instructions for [Stanford WBC][stanford-wbc]. Adapt the
URLs and paths when pushing/pulling upstream [UT Autsin
WBC][utaustin-wbc].

[git-subtree]: https://github.com/apenwarr/git-subtree
[stanford-wbc]: https://github.com/poftwaresatent/stanford_wbc
[utaustin-wbc]: https://github.com/poftwaresatent/utaustin_wbc_opspace

- **Do not mix** commits to `wbc_core/src` with commits to other
  locations of the `whole_body_control` stack. Well, this is not a
  super strict requirement, but it avoids confusing spurious
  commits upstream...

- In order to **pull updates** from upstream, follow these steps:

  1. If you don't have it yet, check out (and optionally "install")
     [git-subtree][].

  2. If you don't yet have the upstream remote in your clone, add it
     like this (adapt the URL to suit your needs, in this example we
     use read-only access):

            git remote add stanford_wbc git://github.com/poftwaresatent/stanford_wbc.git

  3. Fetch the upstream changes and merge them into `wbc_core/src`
     using `git-subtree`:

            git fetch stanford_wbc
            git subtree merge -P wbc_core/src stanford_wbc/master

     Here we assume you have `git-subtree` properly
     installed. Otherwise, replace "`git subtree`" with
     "`/path/to/git-subtree.sh`" in the command above.

- In order to **push changes** upstream, follow these steps:

  1. If you don't have it yet, check out (and optionally "install")
     [git-subtree][]

  2. If you don't yet have the upstream remote in your clone, add it
     like this (here we'll need write access, so maybe adapt the URL
     to point to a fork):

            git remote add stanford_wbc git@github.com:poftwaresatent/stanford_wbc.git

  3. Use a temporary branch to split out the changes and push them upstream:

            git subtree split -P wbc_core/src -b foo
            git push stanford_wbc foo:master
            git branch -D foo

    Again, this assumes you have properly installed
    `git-subtree`. Otherwise, say "`/path/to/git-subtree.sh`" instead.


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
