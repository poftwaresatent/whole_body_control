# A copy of the header that came in from sensor_msgs/JointState.

Header header


# If something goes wrong, this will be false and you can read the
# reason why in errstr.

bool ok
string errstr


# Name of the link which is the root of the TAO model. This link is
# considered fixed wrt the inertial frame when computing the
# joint-space dynamics.

string root_name


# The names of the joints for which this model has been computed.

string[] name


# The joint positions, in the same order as the joint names. This is
# just a subset of the sensor_msgs/JointState/position.

float64[] position


# The joint velocities, in the same order as the joint names. This is
# just a subset of the sensor_msgs/JointState/velocity.

float64[] velocity


# The joint torques due to gravity. All joints which are mechanically
# gravity compensated will simply have a zero here.

float64[] gravity


# The joint torques due to Coriolis and centrifugal effects. Beware
# that using this in your controller can lead to instabilities due to
# modelling errors and time lag.

float64[] coriolis_centrifugal


# Mass-inertia matrix. This is a symmetric matrix of dimension
# position.size() squared, stuffed into one long array.

float64[] mass_inertia


# Inverse of the mass-inertia matrix.

float64[] inv_mass_inertia
