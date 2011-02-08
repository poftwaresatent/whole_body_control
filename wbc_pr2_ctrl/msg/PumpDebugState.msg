string[] joint_name
float64[] position
float64[] velocity

# force is the measured_effort data from the
# pr2_mechanism_model::JointState.  We plast it out so that you can
# compare the desired with the actually commanded torque. The latter
# can be found in the PumpDebugCommand message.

float64[] force

# command_delta is a heuristic measuring badly we saturate the
# motors. Look at src/pump_plugin.cpp for how it is computed. It boils
# down to keeping some history of the desired commanded effort
# (e.g. the 10 last commands), comparing the measured effort with
# that, taking the closest matching one (because we are optimists),
# and computing the relative error between the two. In case the
# commanded effort is close to zero, we use the absolute error
# instead, to avoid dividing by zero.

float64[] command_delta
