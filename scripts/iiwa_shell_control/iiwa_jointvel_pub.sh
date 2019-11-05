#/bin/bash

rostopic pub -1 /iiwa/command/JointVelocity iiwa_msgs/JointVelocity "{'header': {'seq': 0, 'stamp':{'secs': 0, 'nsecs': 0}, 'frame_id': ''}, 'velocity': {'a1': 0.0, 'a2': 0.0, 'a3': 0.0, 'a4': 0.0, 'a5': 0.0, 'a6': 0.25, 'a7': 0.0}}"
#sleep 1
echo "Done"
