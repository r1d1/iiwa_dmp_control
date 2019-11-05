#/bin/bash

rostopic pub -1 /iiwa/dmp_goal imagine_common/DMPGoal "{cart_pose: {'header': {'seq': 0, 'stamp':{'secs': 0, 'nsecs': 0}, 'frame_id': 'iiwa_link_0'}, 'pose': {'position': {'x': 0.6, 'y': 0.0, 'z': 0.25}, 'orientation': {'x': 0.0, 'y': 0.797, 'z': 0.454, 'w': 0.399}}}, 'duration': {'data': {'secs': 5.0, 'nsecs' : 0.}}}"
#rostopic pub -1 /iiwa/dmp_goal geometry_msgs/PoseStamped "{'header': {'seq': 0, 'stamp':{'secs': 0, 'nsecs': 0}, 'frame_id': 'iiwa_link_0'}, 'pose': {'position': {'x': 0.40, 'y': -0.35, 'z': 0.05}, 'orientation': {'x': 0.0, 'y': 1.0, 'z': 0.0, 'w': 0.0}}}"
echo "Done"
