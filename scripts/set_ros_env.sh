export ROS_DOMAIN_ID=100
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Cyclone DDS on loopback, for EVERY process started from this environment (the
# robot driver, rqt, robot_control.py, the overlay). The tracking stack runs on
# this config (see MT_Project/launch_fpose_stack.sh and config/cyclonedds_local.xml);
# a process without it lives on the wired NIC and neither sees nor is seen by
# the rest -- the symptom is crisp_py timing out on /joint_states while the
# driver is demonstrably publishing. Set it explicitly to override.
if [ -z "${CYCLONEDDS_URI:-}" ]; then
  _cdds="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." 2>/dev/null && pwd)/config/cyclonedds_local.xml"
  if [ -f "$_cdds" ]; then export CYCLONEDDS_URI="file://$_cdds"; fi
  unset _cdds
fi
