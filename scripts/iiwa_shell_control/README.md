# Collection of scripts for IIWA control

These are used and example scripts to command IIWA from the available ROS topics. [DEPR] indicates old scripts that should be carefully checked before running due to recent changes in the robot setup.

 - `iiwa_dmp_goal.sh` sends an imagine\_common/DMPGoal (cartesian posem motion duration) to be used with the dmp controller from this package.

 - `iiwa_pub.sh` publishes a cartesian pose.

 - [DEPR] `iiwa_jointpos_pub.sh` sets a trajectory of joint positions with user control to next waypoint 

 - [DEPR] `iiwa_jointpos_sequence_for_hdd_views.sh` same as `iiwa_jointpos_pub.sh` with a trajectory that rotates around the HDD with nerian

 - [DEPR] `iiwa_jointpos_sequence2_for_hdd_views.sh` same as `iiwa_jointpos_pub.sh` with a trajectory that rotates around the HDD with nerian

 - `iiwa_jointpos_sequence3_calib.sh` same as `iiwa_jointpos_pub.sh` with a trajectory that rotates around the calibration pattern with nerian

 - [DEPR] `iiwa_jointpos_topview.sh` goes to top view for nerian

 - `iiwa_mode_manualguidance.sh` sets mode, impedance and damping values to allow manual guidance of the robot

 - `iiwa_mode_positioncontrol.sh` sets position control, resetting the original stiffness (to disable manual guidance)

 - `iiwa_jointvel_pub.sh` sets a joint velocity to desired value

 - `iiwa_stop.sh` sets all joint velocities to 0.0

