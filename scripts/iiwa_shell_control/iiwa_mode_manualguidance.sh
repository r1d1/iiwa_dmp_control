#!/bin/bash

rosservice call /iiwa/configuration/configureSmartServo "{control_mode: 1, joint_impedance: {joint_stiffness: {a1: 0.0, a2: 0.0, a3: 0.0, a4: 0.0, a5: 0.0, a6: 0.0, a7: 0.0}, joint_damping: {a1: 0.0, a2: 0.0, a3: 0.0, a4: 0.0, a5: 0.0, a6: 0.0, a7: 0.0}}}"

#joint_impedance:
#  joint_stiffness: {a1: 0.0, a2: 0.0, a3: 0.0, a4: 0.0, a5: 0.0, a6: 0.0, a7: 0.0}
#  joint_damping: {a1: 0.0, a2: 0.0, a3: 0.0, a4: 0.0, a5: 0.0, a6: 0.0, a7: 0.0}
#cartesian_impedance:
#  cartesian_stiffness: {x: 0.0, y: 0.0, z: 0.0, a: 0.0, b: 0.0, c: 0.0}
#  cartesian_damping: {x: 0.0, y: 0.0, z: 0.0, a: 0.0, b: 0.0, c: 0.0}
#  nullspace_stiffness: 0.0
#  nullspace_damping: 0.0
#desired_force: {cartesian_dof: 0, desired_force: 0.0, desired_stiffness: 0.0}
#sine_pattern: {cartesian_dof: 0, frequency: 0.0, amplitude: 0.0, stiffness: 0.0}
#limits:
#  max_path_deviation: {x: 0.0, y: 0.0, z: 0.0, a: 0.0, b: 0.0, c: 0.0}
#  max_control_force: {x: 0.0, y: 0.0, z: 0.0, a: 0.0, b: 0.0, c: 0.0}
#  max_control_force_stop: false
#  max_cartesian_velocity: {x: 0.0, y: 0.0, z: 0.0, a: 0.0, b: 0.0, c: 0.0}"
