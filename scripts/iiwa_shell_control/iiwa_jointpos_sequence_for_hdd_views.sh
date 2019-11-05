#/bin/bash

# =====================================================
rostopic pub -1 /iiwa/command/JointPosition iiwa_msgs/JointPosition "{'header': {'seq': 0, 'stamp':{'secs': 0, 'nsecs': 0}, 'frame_id': ''}, 'position': {'a1': 0.8, 'a2': 0.60, 'a3': -0.200, 'a4': -1.02, 'a5': 0.5, 'a6': 1.37, 'a7': 2.4}}"

# =====================================================
read  -n 1 -p "Go to next pose (y)\r or stop execution (q)\r or ignore waypoint (any other) ?" input
#input="a"

if [ "$input" = "q" ]; then
    exit 1
elif [ "$input" = "y" ]; then
    # 1st waypoint -- OK!
    rostopic pub -1 /iiwa/command/JointPosition iiwa_msgs/JointPosition "{'header': {'seq': 0, 'stamp':{'secs': 0, 'nsecs': 0}, 'frame_id': ''}, 'position': {'a1': 0.74, 'a2': 0.83, 'a3': -0.300, 'a4': -1.02, 'a5': 0.5, 'a6': 1.37, 'a7': 2.4}}"
else
    echo "ignoring waypoint"
fi

# =====================================================
read  -n 1 -p "Go to next pose (y)\r or stop execution (q)\r or ignore waypoint (any other) ?" input

if [ "$input" = "q" ]; then
    exit 1
elif [ "$input" = "y" ]; then
    # 2nd waypoint
    rostopic pub -1 /iiwa/command/JointPosition iiwa_msgs/JointPosition "{'header': {'seq': 0, 'stamp':{'secs': 0, 'nsecs': 0}, 'frame_id': ''}, 'position': {'a1': 0.215, 'a2': 0.664, 'a3': -0.190, 'a4': -1.097, 'a5': 0.739, 'a6': 1.465, 'a7': 1.978}}"
else
    echo "ignoring waypoint"
fi

# =====================================================
read  -n 1 -p "Go to next pose (y)\r or stop execution (q)\r or ignore waypoint (any other) ?" input
if [ "$input" = "q" ]; then
    exit 1
elif [ "$input" = "y" ]; then
    # 3rd waypoint
    rostopic pub -1 /iiwa/command/JointPosition iiwa_msgs/JointPosition "{'header': {'seq': 0, 'stamp':{'secs': 0, 'nsecs': 0}, 'frame_id': ''}, 'position': {'a1': -0.239, 'a2': 0.808, 'a3': -0.190, 'a4': -1.044, 'a5': 1.147, 'a6': 1.81, 'a7': 1.687}}"
else
    echo "ignoring waypoint"
fi
# =====================================================
read  -n 1 -p "Go to next pose (y)\r or stop execution (q)\r or ignore waypoint (any other) ?" input
if [ "$input" = "q" ]; then
    exit 1
elif [ "$input" = "y" ]; then
    # 3rd waypoint
    rostopic pub -1 /iiwa/command/JointPosition iiwa_msgs/JointPosition "{'header': {'seq': 0, 'stamp':{'secs': 0, 'nsecs': 0}, 'frame_id': ''}, 'position': {'a1': -1.12, 'a2': 1.328, 'a3': 0.325, 'a4': -1.140, 'a5': 1.611, 'a6': 2.076, 'a7': 1.590}}"
else
    echo "ignoring waypoint"
fi
# =====================================================
# last useful:
# rostopic pub -1 /iiwa/command/JointPosition iiwa_msgs/JointPosition "{'header': {'seq': 0, 'stamp':{'secs': 0, 'nsecs': 0}, 'frame_id': ''}, 'position': {'a1': -0.16, 'a2': 0.73, 'a3': -0.35, 'a4': -1.45, 'a5': 0.68, 'a6': 1.17, 'a7': -0.2}}"
#sleep 1
echo "Done"
