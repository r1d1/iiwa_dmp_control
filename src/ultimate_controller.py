#!/usr/bin/env python

'''
    Ultimate controller imagine; control arm (DMP), gripper, suction
'''

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray, Int32MultiArray, MultiArrayDimension, Bool, Int32, String, Time
from std_srvs.srv import Empty
from trajectory_msgs.msg import JointTrajectory 
from control_msgs.msg import JointTrajectoryControllerState
from imagine_common.msg import UltimateDMPGoal
import numpy as np

from colors_toolbox import DMP as ctd
#import actionlib
from iiwa_dmp_control.srv import *
from caros_control_msgs.srv import GripperMoveQ 
from caros_control_msgs.msg import GripperState
import pandas as pd
import matplotlib.pyplot as plt
from sklearn import preprocessing

class UltimateController():
    def __init__(self, db, saving, plotting):
        
        print("Should we store generated trajectory: "+str(saving))
        self.saved_traj = []
        self.plotting = plotting

        # --- 
        # DMP from BOUN toolbox / it needs to be initialized
        # for the purpose of testing, let's load that with a hardcoded traj:
        lengths = []
        raw_traj_data = []
        traj_data = []
        self.dmptype = None
        # default motion duration is 1 second
        self.motion_duration = 1.0

        if db is None:
            print("No demonstration data provided ; can't train and generate dmp. Exiting ...")
            exit()
        else:
            datafile=db
            for f in datafile:
                raw_traj_data.append(pd.read_csv(f, index_col=0))
                lengths.append(len(raw_traj_data[-1]))

        nb_exp = len(raw_traj_data)
        #print(raw_traj_data[0]['scaled_time'])
        print(nb_exp, " demonstrations provided")
        self.miniLength = max(lengths)

        print("Max data length", self.miniLength)
        # interpolate to have same size data
        # raw_traj_data : 1 element per exp, e.g. 5 elements
        self.motion_duration = 4.35
        indices = np.linspace(0., self.motion_duration, num=self.miniLength)
        print("indices len:",len(indices))
        for d in raw_traj_data:
            # each d is a demonstration
            steps = len(d)
            # Just scale the time
            d['scaled_time'] = indices 
            traj_data.append(d)

        # Create trajectories to encode in DMP
        self.dataset = np.array([traj_data[e].values for e in np.arange(len(traj_data))])

        # ------------------------ end of dmp stuff
        # Degrees of freedom for DMP 
        # cartesian dmp + gripper + suck, thus (3+4)+1+1 or 7 dof
        self.DoF = self.dataset[0,0,1:].shape[0]
        print("Found",self.DoF,"DOF in data.")
        
        self.dataset2 = np.reshape(self.dataset, (len(traj_data)*len(traj_data[0]), (self.DoF+1)))
        print("Dataset2 shape:", self.dataset2.shape, len(traj_data), len(traj_data[0]))
        K = 150 * np.ones(self.DoF)
        self.bf_number = 20

        self.dmp = ctd.DMP(self.bf_number, K, nb_exp, self.dataset2)
        
        # ------- Plotting
        self.fig = None
        from palettable.colorbrewer.qualitative import Set3_12
        self.colors = Set3_12.mpl_colors
        if self.plotting:
            self.fig = plt.figure(figsize=(12,12))
            c=0
            # plot data per DoF
            # transpose order data as DoF, Time, exp
            for d in (self.dataset.transpose())[1:]:
                self.ax = self.fig.add_subplot(int(self.DoF/2)+1,2,c+1)
                self.ax.plot(indices, d[:,:], c=self.colors[c], linestyle='--', label="demo")
                c+=1
       
            min_traj = np.amin(self.dataset[:,:,1:], axis=0)
            max_traj = np.amax(self.dataset[:,:,1:], axis=0)
            mean_init = np.mean(self.dataset[:,0,1:], 0)
            mean_goal = np.mean(self.dataset[:,-1,1:], 0)

            # Generate sample data from demos
            gen_data = self.generate_traj(mean_init, mean_goal, self.motion_duration)
            
            for i in np.arange(self.DoF):
                self.ax = self.fig.add_subplot(int(self.DoF/2)+1,2,i+1)
                mapped = np.interp(gen_data[:,0], np.arange(len(gen_data[:,1+i])), gen_data[:,1+i])
                self.ax.plot(gen_data[:,0], gen_data[:,1+i], c=self.colors[i], lw=2, label="dmp")
                self.ax.set_xlabel("time")
                self.ax.set_ylabel("value")
                self.ax.set_title("dimension "+str(i))
                self.ax.legend(loc=0, ncol=2, prop={'size': 10})
            plt.tight_layout()
            exp_nb = ''
            for d in args.demo:
                nb = d.split("/")[-1].split(".")[0].split("_")[-1]
                exp_nb += nb
            #self.fig.savefig("/home/erwan/Documents/IMAGINE/IMAGINE-2018-TR-ADES/figures/push_from_right_dmp_"+exp_nb+".pdf", dpi=300)
            plt.show()
        # ---------------------

        self.timer = rospy.Timer(rospy.Duration(0.05), self.timer_cb)
        self.exec_traj = False

        # State variables
        self.cart_arm_state = PoseStamped()
        self.dest_reached = False
        self.gripper_state = GripperState()
        #self.gripper_state = 0.20
        self.gripper_limits = {'min': 0.002, 'max': 0.032}
        self.suction_limits = {'min': 0, 'max': 1024}
        self.suction_state = Int32() 

        # Manage DMP parameters
        self.set_param_serv = rospy.Service("/dmp_controller/set_params", SetParams, self.set_params)
        self.get_param_serv = rospy.Service("/dmp_controller/get_params", GetParams, self.get_params)

        # get arm state
        self.cart_arm_state_sub = rospy.Subscriber("/iiwa/state/CartesianPose", PoseStamped, self.cart_arm_state_cb)
        self.cart_arm_state_sub = rospy.Subscriber("/iiwa/state/DestinationReached", Time, self.dest_reached_cb)
        self.gripper_state_sub = rospy.Subscriber("/caros_schunkpg70/caros_gripper_service_interface/gripper_state", GripperState, self.gripper_state_cb)
        self.suctiontool_sub = rospy.Subscriber("/suction_gripper/state", Int32, self.suction_state_cb)
        
        # outputs
        self.cart_arm_command_pub = rospy.Publisher("iiwa/command/CartesianPose", PoseStamped, queue_size=1)
        # These should be service proxies
        self.gripper_proxy = rospy.ServiceProxy("/caros_schunkpg70/caros_gripper_service_interface/move_q", GripperMoveQ)
        self.suctiontool_proxy = rospy.ServiceProxy("~ss_service", Empty)
        #self.gripper_command_pub = rospy.Publisher("/iiwa/command/CartesianPose", PoseStamped, queue_size=1)
        self.suctiontool_command_pub = rospy.Publisher("/suction_gripper/strength", Int32, queue_size=1)
        
        self.cart_goal_sub = rospy.Subscriber("iiwa/dmp_goal", UltimateDMPGoal, self.goal_cb)

        print("Controller initialized")
        
    def dest_reached_cb(self, msg):
        self.dest_reached = True

    def cart_arm_state_cb(self, msg):
        # Position in joint space
        self.cart_arm_state = msg

    def gripper_state_cb(self, msg):
        #print(msg)
        self.gripper_state = msg
    
    def suction_state_cb(self, msg):
        #print(msg.data)
        self.suction_state = msg

    # service implementation
    def set_params(self, req):
        # test on req structure : req.
        print "Setting DMP params" #,len(req.params.layout.dim)
        #d = req.params.layout.dim[0]
        offset = 0
        data_range = 0
        for d in req.params.layout.dim:
            # print d.label, d.size
            data_range = d.size
            if d.label == 'K':
                # print "shape current data:",(self.dmp.K).shape
                # print "shape req data:", np.array(req.params.data[offset:(offset+data_range)]).shape
                self.dmp.K = np.array(req.params.data[offset:(offset+data_range)]).reshape(self.DoF, self.DoF)
            elif d.label == 'D':
                # print "shape current data:",(self.dmp.D).shape
                # print "shape req data:", np.array(req.params.data[offset:(offset+data_range)]).shape
                self.dmp.D = np.array(req.params.data[offset:(offset+data_range)]).reshape(self.DoF, self.DoF)
            elif d.label == 'weights':
                # print "shape current data:",(self.dmp.weights).shape
                # print "shape req data:", np.array(req.params.data[offset:(offset+data_range)]).shape
                self.dmp.weights = np.array(req.params.data[offset:(offset+data_range)]).reshape(self.bf_number, self.DoF)
            elif d.label == 'psiMatrix':
                # Ignore that and compute it from the function
                pass
                # print "shape current data:",(self.dmp.psiMatrix).shape
                # print len(self.dataset)
                # print "shape req data:", np.array(req.params.data[offset:(offset+data_range)]).shape
                # print "shape req data:", np.array(req.params.data[offset:(offset+data_range)]).shape[0] / self.bf_number
                #self.dmp.psiMatrix = np.array(req.params.data[offset:(offset+data_range)]).reshape(len(self.dataset), self.bf_number)
            else:
                print "\033[31mUnknown label\033[39m"
            offset += data_range
            data_range = 0
            data_range = d.size

        self.dmp.sumBFs, self.dmp.psiMatrix = self.dmp.getBasisFunctions()
        #self.dmp.weights = self.dmp.learnWeights() # shouldn't be necessary as they are provided in ADES
        return True

    def get_params(self, req):
        offset = 0
        params_values = []

        params = Float64MultiArray()
        params.layout.data_offset = offset

        params.layout.dim.append(MultiArrayDimension())
        params.layout.dim[0].label='K'
        params.layout.dim[0].size = len(self.dmp.K[0]) * len(self.dmp.K)
        params.layout.dim[0].stride = 0
        params_values.append([l for v in self.dmp.K for l in v])
        
        params.layout.dim.append(MultiArrayDimension())
        params.layout.dim[1].label='D'
        params.layout.dim[1].size = len(self.dmp.D[0]) * len(self.dmp.D)
        params.layout.dim[1].stride=0
        params_values.append([l for v in self.dmp.D for l in v])
        
        params.layout.dim.append(MultiArrayDimension())
        params.layout.dim[2].label='weights'
        params.layout.dim[2].size = len(self.dmp.weights[0]) * len(self.dmp.weights)
        params.layout.dim[2].stride=0
        params_values.append([l for v in self.dmp.weights for l in v])
        
        params.layout.dim.append(MultiArrayDimension())
        params.layout.dim[3].label='psiMatrix'
        params.layout.dim[3].size = len(self.dmp.psiMatrix[0]) * len(self.dmp.psiMatrix)
        params.layout.dim[3].stride=0
        params_values.append([l for v in self.dmp.psiMatrix for l in v])

        params_values = [v for l in params_values for v in l]

        params.data = params_values
        return params 

    # TODO: UPDATE X0 FOR POSE+GRIPPER+SUCKING
    def generate_traj(self, x0, g0, duration):
        #g0 = pose, gripper_pose, suction
        # Set force feedback to zero for this example
        zeta = 0 # force feedback at

        # Set delta time for numerical differentiation 
        nb_target_points = self.miniLength
        self.motion_duration = float(duration)
        dt = float(duration)/nb_target_points
        des_tau = float(duration)

        if len(g0) != 9:
            print("Goal has a wrong size ... giving up.")
            return 
        #g0 = np.array([pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w])
    
        current_pos = x0 
        current_vel = np.zeros(self.DoF)  # Initially the robot arm is at rest
        
        # We have to start at dt rather than 0 as traj already contain x0
        t = dt 
        #t = 0. 
        i = 0 ## burayi da duzelt
        #traj = np.zeros(3)
        # Adding x0 at traj point list
        traj = x0
        #joint_traj = np.zeros(7)
        time_traj = np.zeros((1,1))

        for i in np.arange(self.miniLength-1):
            # Execute DMP
            desired_pos, desired_vel  = self.dmp.executeDMP(t, dt, des_tau, x0, g0, current_pos, current_vel,zeta)
            
            # Execute the robot arm to the desired position with desired velocities
            current_pos = desired_pos
            current_vel = desired_vel
            
            # Store the current positions
            traj = np.row_stack((traj, current_pos))
            time_traj = np.row_stack((time_traj, t))
            
            i += 1 ## plot_positiondan alabiliriz
            t += dt

        stamped_traj = np.column_stack((time_traj, traj))

        #self.saved_traj.append(traj)
        self.saved_traj.append(stamped_traj)
        #return traj
        return stamped_traj

    # now traj callbacks should convert the cart goal into a joint trajectory
    def goal_cb(self, msg):
        print("Goal received")
        #print(msg)
        # MSG is a cartesian goal pose + gripper state + suction tool + duration (UltimateDMPGoal.msg)
        print(self.gripper_state.q.data)
        print(self.suction_state)
        x0 = np.array([self.cart_arm_state.pose.position.x, self.cart_arm_state.pose.position.y, self.cart_arm_state.pose.position.z, self.cart_arm_state.pose.orientation.x, self.cart_arm_state.pose.orientation.y, self.cart_arm_state.pose.orientation.z, self.cart_arm_state.pose.orientation.w, self.gripper_state.q.data[0], self.suction_state.data])
        
        duration = msg.duration
        g0 = np.array([msg.cart_pose.pose.position.x, msg.cart_pose.pose.position.y, msg.cart_pose.pose.position.z, msg.cart_pose.pose.orientation.x, msg.cart_pose.pose.orientation.y, msg.cart_pose.pose.orientation.z, msg.cart_pose.pose.orientation.w, msg.gripper_state.q.data[0], msg.suction_state.data])
        # Compute DMP traj from received goal
        print(x0)
        print(g0)
        gen_data = self.generate_traj(x0, g0,  duration.data.to_sec()) # result of IK
        self.traj = gen_data
        #self.traj[:,-1] = np.where(self.traj[:,-1] < 0.5, 0,1)
        self.traj[:,-2] = np.clip(self.traj[:,-2], self.gripper_limits['min'], self.gripper_limits['max'])
        self.traj[:,-1] = np.clip(self.traj[:,-1], self.suction_limits['min'], self.suction_limits['max'])
        print("\033[31mGo Signal disabled for testing\033[39m")
        self.exec_traj = False
       
        if self.plotting:
            self.fig = plt.figure(figsize=(12,12))
            for i in np.arange(self.DoF):
                self.ax = self.fig.add_subplot(int(self.DoF/2)+1,2,i+1)
                mapped = np.interp(gen_data[:,0], np.arange(len(gen_data[:,1+i])), gen_data[:,1+i])
                self.ax.plot(gen_data[:,0], gen_data[:,1+i], c=self.colors[i], lw=2, label="dmp")
                self.ax.set_xlabel("time")
                self.ax.set_ylabel("value")
                self.ax.set_title("dimension "+str(i))
                self.ax.legend(loc=0, ncol=2, prop={'size': 10})
            plt.tight_layout()
            plt.show()

        print("Arm traj")
        print(self.exec_traj)

    def timer_cb(self, msg):
        if self.exec_traj:
            print("Exec DMP")
            offset = 0
            if len(self.traj[0]) == 8:
                print("stamped traj")
                offset = 1
            count = 0
            prev = 0
            #start_time = rospy.Time.now()
            for d in self.traj:
                cmd = PoseStamped()
                grip_cmd = Int()
                suck_cmd = Empty()
                cmd.pose.position.x = d[offset+0]
                cmd.pose.position.y = d[offset+1]
                # Adding boundary value on z to avoid collision with the table
                cmd.pose.position.z = d[offset+2] if d[offset+2] > 0.001 else 0.001
                cmd.pose.orientation.x = d[offset+3]
                cmd.pose.orientation.y = d[offset+4]
                cmd.pose.orientation.z = d[offset+5]
                cmd.pose.orientation.w = d[offset+6]

                #cmd.header.stamp = start_time + rospy.Duration.from_sec(d[0])
                cmd.header.stamp = rospy.Time.now()
                cmd.header.frame_id = 'iiwa_link_0'
                #cmd.header.seq = count
                timeout = 0.1
                minimal_walltime_timeout = 0.1
                try:
                    self.cart_arm_command_pub.publish(grip_cmd)
                    self.gripper_proxy.publish(cmd)
                    self.suctiontool_proxy.publish(suck_cmd)
                    rospy.sleep(d[0] - prev)
                    prev = d[0]
                    # Waiting is not needed: it creates shaky movements 
                    #while not self.dest_reached:
                    #    pass
                    #self.dest_reached = False
                except Exception as e:
                    print(e)

            self.exec_traj = False
            print("Done")

if __name__ == "__main__":
    import argparse
    # Argument handling
    parser = argparse.ArgumentParser(description="DMP controller")
    #parser.add_argument('--demo', type=str, help="dmp demo data")
    #parser.add_argument('--demo', type=argparse.FileType('r'), nargs='+', help="dmp demo data")
    parser.add_argument('--demo', type=str, nargs='+', help="dmp demo data")
    #parser.add_argument('--robot', type=str, default="/simulation", help="simulation or real")
    #parser.add_argument('--type', type=str, default="cart", help="cart or joint: cartesian or joint dmp")
    parser.add_argument('--save', action='store_true', help="should we save generated traj ?")
    parser.add_argument('--plot', action='store_true', help="should we display trajectories (demo, mean, generated) ?")
    args = parser.parse_args()
    rospy.init_node("iiwa_cart_dmp")
    print(args.demo)

    dcc = UltimateController(db=args.demo, saving=args.save, plotting=args.plot)

    rospy.spin()

    if args.save:
        j = 0
        for tr in dcc.saved_traj:
            gtraj = open("./generated_traj/traj_"+str(j)+".txt",'w')
            listification=[[str(i) for i in l] for l in tr]
            gtraj.write( "\n".join([", ".join(l) for l in listification]) )
            gtraj.close()
            j+=1

    print "Controller stopping"

