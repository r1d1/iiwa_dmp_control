erwan.renaudo@c703i-lab-ws3:~/catkin_ws$ rostopic pub -1 /iiwa/dmp_goal imagine_common/UltimateDMPGoal "cart_pose:
  header:
    seq: 0
    stamp:
      secs: 0
      nsecs: 0
    frame_id: ''
  pose:
    position:
      x: 0.0
      y: 0.0
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 0.0
gripper_state:
  header:
    seq: 0
    stamp: {secs: 0, nsecs: 0}
    frame_id: ''
  q:
    data: [0.25]
  dq:
    data: [0]
  force:
    data: [0]
  is_moving: false
  is_blocked: false
  is_stopped: false
  e_stopped: false
suction_state: {data: 256}
duration:
  data: {secs: 5.0, nsecs: 0}" 

