#/bin/bash

# Go to small calib pattern
#rostopic pub -1 /iiwa/command/CartesianPose geometry_msgs/PoseStamped "{'header': {'seq': 0, 'stamp':{'secs': 0, 'nsecs': 0}, 'frame_id': 'iiwa_link_0'}, 'pose': {'position': {'x': 0.559, 'y': -0.006, 'z': -0.11}, 'orientation': {'x': 1.0, 'y': 0.0, 'z': 0.0, 'w': 0.0}}}"
# some out of pattern from camera view position
# close to HDD pcb :
#rostopic pub -1 /iiwa/command/CartesianPose geometry_msgs/PoseStamped "{'header': {'seq': 0, 'stamp':{'secs': 0, 'nsecs': 0}, 'frame_id': 'iiwa_link_0'}, 'pose': {'position': {'x': 0.61, 'y': -0.030, 'z': 0.0915}, 'orientation': {'x': 0.273, 'y': 0.941, 'z': 0.195, 'w': -0.0050}}}"
rostopic pub -1 /iiwa/command/CartesianPose geometry_msgs/PoseStamped "{'header': {'seq': 0, 'stamp':{'secs': 0, 'nsecs': 0}, 'frame_id': 'iiwa_link_0'}, 'pose': {'position': {'x': 0.55, 'y': 0.007, 'z': 0.416}, 'orientation': {'x': -0.615, 'y': -0.364, 'z': -0.411, 'w': 0.569}}}"
# Last useful pose
# rostopic pub -1 /iiwa/command/CartesianPose geometry_msgs/PoseStamped "{'header': {'seq': 0, 'stamp':{'secs': 0, 'nsecs': 0}, 'frame_id': 'iiwa_link_0'}, 'pose': {'position': {'x': 0.45, 'y': -0.25, 'z': 0.2}, 'orientation': {'x': 0.273, 'y': 0.941, 'z': 0.195, 'w': -0.0050}}}"
#sleep 1
echo "Done"
