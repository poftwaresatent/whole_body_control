ROS stack for Stanford Whole-Body Controller
==============================================

Installation
------------

1. install [ROS](http://www.ros.org/wiki/ROS/Installation)
   (you will need PR2-specific stacks, including PR2 simulator)

2. clone the stack repos into your `ROS_PACKAGE_PATH`, e.g. in `~/ros/stacks`
   - cd ~/ros/stacks
   - git clone git://github.com/poftwaresatent/whole_body_control.git

3. use `git submodule` to pull in the core STanford_WBC library code
   - cd whole_body_control
   - git submodule init
   - git submodule update

4. build it using `rosmake`
   - rosmake

