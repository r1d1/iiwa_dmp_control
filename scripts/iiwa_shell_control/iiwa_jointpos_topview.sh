#/bin/bash

# =====================================================
rostopic pub -1 /iiwa/command/JointPosition iiwa_msgs/JointPosition "{'header': {'seq': 0, 'stamp':{'secs': 0, 'nsecs': 0}, 'frame_id': ''}, 'position': {'a1': -0.43, 'a2': 0.99, 'a3': -0.18, 'a4': -1.03, 'a5': 1.42, 'a6': 2.09, 'a7': 1.44}}"

echo "Done"
