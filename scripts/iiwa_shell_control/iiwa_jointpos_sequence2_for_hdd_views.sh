#/bin/bash

# =====================================================
read  -n 1 -p "Go to next pose (y) or stop execution (q) or ignore waypoint (any other) ?" input

if [ "$input" = "q" ]; then
    exit 1
elif [ "$input" = "y" ]; then
    # 1st waypoint -- OK!
    rostopic pub -1 /iiwa/command/JointPosition iiwa_msgs/JointPosition "{'header': {'seq': 0, 'stamp':{'secs': 0, 'nsecs': 0}, 'frame_id': ''}, 'position': {'a1': -1.20, 'a2': 1.092, 'a3': 0.984, 'a4': -1.380, 'a5': 0.675, 'a6': 1.656, 'a7': 1.612}}"
else
    echo "ignoring waypoint"
fi

# =====================================================
read  -n 1 -p "Go to next pose (y) or stop execution (q) or ignore waypoint (any other) ?" input

if [ "$input" = "q" ]; then
    exit 1
elif [ "$input" = "y" ]; then
    # 2nd waypoint
    rostopic pub -1 /iiwa/command/JointPosition iiwa_msgs/JointPosition "{'header': {'seq': 0, 'stamp':{'secs': 0, 'nsecs': 0}, 'frame_id': ''}, 'position': {'a1': -0.740, 'a2': 0.674, 'a3': 0.984, 'a4': -1.784, 'a5': 0.404, 'a6': 0.858, 'a7': 1.848}}"
else
    echo "ignoring waypoint"
fi

# =====================================================
read  -n 1 -p "Go to next pose (y) or stop execution (q) or ignore waypoint (any other) ?" input
if [ "$input" = "q" ]; then
    exit 1
elif [ "$input" = "y" ]; then
    # 3rd waypoint
    rostopic pub -1 /iiwa/command/JointPosition iiwa_msgs/JointPosition "{'header': {'seq': 0, 'stamp':{'secs': 0, 'nsecs': 0}, 'frame_id': ''}, 'position': {'a1': -0.039, 'a2': 0.966, 'a3': -0.1799, 'a4': -0.7297, 'a5': 0.3707, 'a6': 0.925, 'a7': -3.05}}"
else
    echo "ignoring waypoint"
fi

# =====================================================
read  -n 1 -p "Go to next pose (y) or stop execution (q) or ignore waypoint (any other) ?" input
if [ "$input" = "q" ]; then
    exit 1
elif [ "$input" = "y" ]; then
    # 4th waypoint
    rostopic pub -1 /iiwa/command/JointPosition iiwa_msgs/JointPosition "{'header': {'seq': 0, 'stamp':{'secs': 0, 'nsecs': 0}, 'frame_id': ''}, 'position': {'a1': -0.392, 'a2': 0.678, 'a3': -0.180, 'a4': -1.309, 'a5': -0.095, 'a6': 0.622, 'a7': -2.254}}"
else
    echo "ignoring waypoint"
fi
# =====================================================

read  -n 1 -p "Go to next pose (y) or stop execution (q) or ignore waypoint (any other) ?" input
if [ "$input" = "q" ]; then
    exit 1
elif [ "$input" = "y" ]; then
    # 5th waypoint
    rostopic pub -1 /iiwa/command/JointPosition iiwa_msgs/JointPosition "{'header': {'seq': 0, 'stamp':{'secs': 0, 'nsecs': 0}, 'frame_id': ''}, 'position': {'a1': -0.3078, 'a2': 0.538, 'a3': -0.180, 'a4': -1.515, 'a5': -0.522, 'a6': 0.854, 'a7': -1.304}}"
else
    echo "ignoring waypoint"
fi
# =====================================================

read  -n 1 -p "Go to next pose (y) or stop execution (q) or ignore waypoint (any other) ?" input
if [ "$input" = "q" ]; then
    exit 1
elif [ "$input" = "y" ]; then
    # 6th waypoint
    rostopic pub -1 /iiwa/command/JointPosition iiwa_msgs/JointPosition "{'header': {'seq': 0, 'stamp':{'secs': 0, 'nsecs': 0}, 'frame_id': ''}, 'position': {'a1': -0.057, 'a2': 0.0288, 'a3': -0.1799, 'a4': -2.0327, 'a5': -0.183, 'a6': 1.617, 'a7': 0.193}}"
else
    echo "ignoring waypoint"
fi
# =====================================================
read  -n 1 -p "Go to next pose (y) or stop execution (q) or ignore waypoint (any other) ?" input
if [ "$input" = "q" ]; then
    exit 1
elif [ "$input" = "y" ]; then
    # 7th waypoint
    rostopic pub -1 /iiwa/command/JointPosition iiwa_msgs/JointPosition "{'header': {'seq': 0, 'stamp':{'secs': 0, 'nsecs': 0}, 'frame_id': ''}, 'position': {'a1': -0.028, 'a2': 0.328, 'a3': -0.134, 'a4': -1.970, 'a5': 0.656, 'a6': 1.526, 'a7': 0.794}}"
else
    echo "ignoring waypoint"
fi
# =====================================================
read  -n 1 -p "Go to next pose (y) or stop execution (q) or ignore waypoint (any other) ?" input
if [ "$input" = "q" ]; then
    exit 1
elif [ "$input" = "y" ]; then
    # 8th waypoint
    rostopic pub -1 /iiwa/command/JointPosition iiwa_msgs/JointPosition "{'header': {'seq': 0, 'stamp':{'secs': 0, 'nsecs': 0}, 'frame_id': ''}, 'position': {'a1': -0.052, 'a2': 0.555, 'a3': -0.180, 'a4': -1.463, 'a5': 1.083, 'a6': 1.551, 'a7': 1.399}}"
else
    echo "ignoring waypoint"
fi
# =====================================================
read  -n 1 -p "Go to next pose (y) or stop execution (q) or ignore waypoint (any other) ?" input
if [ "$input" = "q" ]; then
    exit 1
elif [ "$input" = "y" ]; then
    # 9th waypoint
    rostopic pub -1 /iiwa/command/JointPosition iiwa_msgs/JointPosition "{'header': {'seq': 0, 'stamp':{'secs': 0, 'nsecs': 0}, 'frame_id': ''}, 'position': {'a1': -0.130, 'a2': 0.539, 'a3': -0.1799, 'a4': -1.487, 'a5': 1.3149, 'a6': 0.922, 'a7': 1.723}}"
else
    echo "ignoring waypoint"
fi
# =====================================================
read  -n 1 -p "Go to next pose (y) or stop execution (q) or ignore waypoint (any other) ?" input
if [ "$input" = "q" ]; then
    exit 1
elif [ "$input" = "y" ]; then
    # 10th waypoint
    rostopic pub -1 /iiwa/command/JointPosition iiwa_msgs/JointPosition "{'header': {'seq': 0, 'stamp':{'secs': 0, 'nsecs': 0}, 'frame_id': ''}, 'position': {'a1': -0.020, 'a2': 0.456, 'a3': -0.1799, 'a4': -1.616, 'a5': 1.420, 'a6': 0.273, 'a7': 1.989}}"
else
    echo "ignoring waypoint"
fi
# =====================================================
read  -n 1 -p "Go to next pose (y) or stop execution (q) or ignore waypoint (any other) ?" input
if [ "$input" = "q" ]; then
    exit 1
elif [ "$input" = "y" ]; then
    # 11th waypoint
    rostopic pub -1 /iiwa/command/JointPosition iiwa_msgs/JointPosition "{'header': {'seq': 0, 'stamp':{'secs': 0, 'nsecs': 0}, 'frame_id': ''}, 'position': {'a1': -0.333, 'a2': 0.497, 'a3': -0.1799, 'a4': -1.586, 'a5': 0.479, 'a6': 0.226, 'a7': -2.765}}"
else
    echo "ignoring waypoint"
fi
# =====================================================
read  -n 1 -p "Go to next pose (y) or stop execution (q) or ignore waypoint (any other) ?" input
if [ "$input" = "q" ]; then
    exit 1
elif [ "$input" = "y" ]; then
    # 12th waypoint
    rostopic pub -1 /iiwa/command/JointPosition iiwa_msgs/JointPosition "{'header': {'seq': 0, 'stamp':{'secs': 0, 'nsecs': 0}, 'frame_id': ''}, 'position': {'a1': -0.362, 'a2': 0.404, 'a3': -0.180, 'a4': -1.717, 'a5': -0.943, 'a6': 0.802, 'a7': -1.024}}"
else
    echo "ignoring waypoint"
fi
# =====================================================
read  -n 1 -p "Go to next pose (y) or stop execution (q) or ignore waypoint (any other) ?" input
if [ "$input" = "q" ]; then
    exit 1
elif [ "$input" = "y" ]; then
    # 13th waypoint
    rostopic pub -1 /iiwa/command/JointPosition iiwa_msgs/JointPosition "{'header': {'seq': 0, 'stamp':{'secs': 0, 'nsecs': 0}, 'frame_id': ''}, 'position': {'a1': -0.0989, 'a2': 0.276, 'a3': -0.180, 'a4': -1.897, 'a5': -0.693, 'a6': 1.425, 'a7': -0.408}}"
else
    echo "ignoring waypoint"
fi
# =====================================================
read  -n 1 -p "Go to next pose (y) or stop execution (q) or ignore waypoint (any other) ?" input
if [ "$input" = "q" ]; then
    exit 1
elif [ "$input" = "y" ]; then
    # 7th waypoint
    rostopic pub -1 /iiwa/command/JointPosition iiwa_msgs/JointPosition "{'header': {'seq': 0, 'stamp':{'secs': 0, 'nsecs': 0}, 'frame_id': ''}, 'position': {'a1': 0.198, 'a2': 0.507, 'a3': -0.216, 'a4': -1.6147, 'a5': -0.017, 'a6': 2.017, 'a7': 0.316}}"
else
    echo "ignoring waypoint"
fi
# =====================================================

echo "Done"
