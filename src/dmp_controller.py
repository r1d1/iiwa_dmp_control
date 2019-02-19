#!/usr/bin/env python
import threading
import rospy

from geometry_msgs.msg import *
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, Int32MultiArray, MultiArrayDimension, Bool, Int32, String
from trajectory_msgs.msg import JointTrajectory 
from control_msgs.msg import JointTrajectoryControllerState
import numpy as np

from colors_toolbox import DMP as ctd
#import actionlib
import pandas as pd
from iiwa_dmp_control.srv import *
import matplotlib.pyplot as plt

class DmpController():
    def __init__(self, db, dmptype, saving, plotting):
        
        print("Should we store generated trajectory: "+str(saving))
        self.saved_traj = []
        self.plotting = plotting
        # --- beginning of dmp stuff
        # DMP from BOUN toolbox / it needs to be initialized
        # for the purpose of testing, let's load that with a hardcoded traj:

        lengths = []
        raw_traj_data = []
        traj_data = []
        #self.dmptype = dmptype
        self.dmptype = None

        if db is None:
            print("No demonstration data provided ; can't train and generate dmp. Exiting ...")
            exit()
        else:
            datafile=db
            for f in datafile:
                raw_traj_data.append(pd.read_csv(f, index_col=0))
                lengths.append(len(raw_traj_data[-1]))

        nb_exp = len(raw_traj_data)
        print nb_exp, " demonstrations provided"
        rescaledData = []
        self.miniLength = max(lengths)

        print "Max data length", self.miniLength
        # interpolate to have same size data
        # raw_traj_data : 1 element per exp, e.g. 5 elements
        indices = np.linspace(0., 1., num=self.miniLength)
        print "indices len:",len(indices)
        for d in raw_traj_data:
            steps = len(d)
            interp_data = pd.DataFrame(columns=list(d.columns.values))
            interp_data['scaled_time'] = indices
            for count in np.arange(7):
                col_name = 'arm_'+self.dmptype+'_pose'+('_wf' if self.dmptype == 'cart' else '')+'_'+str(count)
                interp_data[col_name] = np.interp(indices, d['scaled_time'], d[col_name])

            traj_data.append(interp_data)

        # Create trajectories to encode in DMP
        self.dataset = np.array([traj_data[e].as_matrix() for e in np.arange(len(traj_data))])

        # we need to scale the duration to a relevant value/pass the desired value
        self.motion_duration = 1.0
        # ------------------------ end of dmp stuff
        # Degrees of freedom for DMP 
        # cartesian or joint dmp, thus 3+4 or 7 dof
        self.DoF = self.dataset[0,0,1:].shape[0]
        print "Found",self.DoF,"DOF in data."
        
        self.dataset2 = np.reshape(self.dataset, (len(traj_data)*len(traj_data[0]), (self.DoF+1)))
        print "Dataset2 shape:", self.dataset2.shape, len(traj_data), len(traj_data[0])
        K = 150 * np.ones(self.DoF)
        self.bf_number = 20

        self.dmp = ctd.DMP(self.bf_number, K, nb_exp, self.dataset2)
        
        # ------- Plotting
        self.fig = None
        if self.plotting:
            self.fig = plt.figure(figsize=(12,12))
            from palettable.colorbrewer.qualitative import Dark2_8
            colors = Dark2_8.mpl_colors
            c=0
            # plot data per DoF
            # transpose order data as DoF, Time, exp
            for d in (self.dataset.transpose())[1:]:
                self.ax = self.fig.add_subplot(int(self.DoF/2)+1,2,c+1)
                self.ax.plot(indices, d[:,:], c=colors[c], linestyle='--', label="demo")
                c+=1
       
            min_traj = np.amin(self.dataset[:,:,1:], axis=0)
            max_traj = np.amax(self.dataset[:,:,1:], axis=0)
            
            mean_init = np.mean(self.dataset[:,0,1:], 0)
            mean_goal = np.mean(self.dataset[:,-1,1:], 0)
            if self.dmptype == 'cart':
                g0 = PoseStamped()
                g0.pose.position.x = mean_goal[0]
                g0.pose.position.y = mean_goal[1]
                g0.pose.position.z = mean_goal[2]
                g0.pose.orientation.x = mean_goal[3]
                g0.pose.orientation.y = mean_goal[4]
                g0.pose.orientation.z = mean_goal[5]
                g0.pose.orientation.w = mean_goal[6]
            else:
                g0 = JointState()
                g0.position = mean_goal
            # Generate sample data from demos
            gen_data = self.generate_traj(mean_init, g0)
            
            for i in np.arange(self.DoF):
                self.ax = self.fig.add_subplot(int(self.DoF/2)+1,2,i+1)
                mapped = np.interp(indices, np.arange(len(gen_data[:,i])), gen_data[:,i])
                self.ax.plot(indices, gen_data[:,i], c=colors[i], lw=2, label="dmp")
                self.ax.fill_between(indices, min_traj[:,i], max_traj[:,i], alpha=0.2)
                self.ax.set_xlabel("time")
                self.ax.set_ylabel("value")
                self.ax.set_title("dimension "+str(i))
                self.ax.legend(loc=0, ncol=2, prop={'size': 10})
            plt.tight_layout()
            exp_nb = ''
            for d in args.demo:
                nb = d.split("/")[-1].split(".")[0].split("_")[-1]
                print nb
                exp_nb += nb
            print exp_nb
            #self.fig.savefig("/home/erwan/Documents/IMAGINE/IMAGINE-2018-TR-ADES/figures/push_from_right_dmp_"+exp_nb+".pdf", dpi=300)
            plt.show()
        # ---------------------

        self.timer = rospy.Timer(rospy.Duration(0.05), self.timer_cb)
        self.exec_traj = False
        #self.traj = {"right":Float64MultiArray, "left":Float64MultiArray}
        self.arm_state = JointTrajectoryControllerState()

        # Manage DMP parameters
        self.set_param_serv = rospy.Service("/dmp_controller/set_params", SetParams, self.set_params)
        self.get_param_serv = rospy.Service("/dmp_controller/get_params", GetParams, self.get_params)

        # DMP are in cartesian space but state/command is anyway in JointSpace
        self.arm_state_sub = rospy.Subscriber("/iiwa/PositionJointInterface_trajectory_controller/state", JointTrajectoryControllerState, self.arm_state_cb)
        self.arm_command_pub = rospy.Publisher("/iiwa/PositionJointInterface_trajectory_controller/command", JointTrajectory, queue_size=1)
        self.cart_goal_sub = rospy.Subscriber("/iiwa/dmp_goal", PoseStamped, self.goal_cb)
        
        # feedback to requesting node
        self.gesture_done_pub = rospy.Publisher("iiwa/gesture_finished", Bool, queue_size=1)
       
        signal_done = Bool()
        signal_done.data = True
        self.gesture_done_pub.publish(signal_done)

        print "%s Controller initialized" % (self.dmptype,)
        
    def arm_state_cb(self, msg):
        #print(msg)
        # Position in joint space
        self.arm_state = msg.actual.positions

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

    #def right_ptp_cb(self, msg):
    #    if msg.data[0]:
    #        self.right_ptp_done.set()
    #    else:
    #        self.right_ptp_done.clear()

    #def left_ptp_cb(self, msg):
    #    if msg.data[0]:
    #        self.left_ptp_done.set()
    #    else:
    #        self.left_ptp_done.clear()

    def generate_traj(self, x0, pose):
        # pose is either PoseStamped or JointState
        # Set force feedback to zero for this example
        zeta = 0 # force feedback at

        # Set delta time for numerical differentiation 
        #nb_target_points = 40
        nb_target_points = self.miniLength
        dt = self.motion_duration/nb_target_points
        des_tau = self.motion_duration

        # Initialize current position and velocities ( e.g. move the arm to the initial position )
        if self.dmptype == "cart":
            #x0 = np.array([self.arm_state.position.x, self.arm_state.position.y, self.arm_state.position.z, self.arm_state.orientation.x, self.arm_state.orientation.y, self.arm_state.orientation.z, self.arm_state.orientation.w])
            g0 = np.array([pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w])
        elif self.dmptype == "joint":
            if len(pose.position) == 7:
                g0 = np.array(pose.position)
            else:
                print "Erroneous joint goal size ; Exiting ..."
                exit()
        else:
            print "Unknown type ; Exiting ..."
            exit()
        #print "Starting position (x0):", x0
        #print "Goal position (g0):", g0
    
        current_pos = x0 
        current_vel = np.zeros(self.DoF)  # Initially the robot arm is at rest
        
        # We have to start at dt rather than 0 as traj already contain x0
        t = dt 
        #t = 0. 
        i = 0 ## burayi da duzelt
        #traj = np.zeros(3)
        traj = x0
        print x0
        joint_traj = np.zeros(7)
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
        return traj

    # now traj callbacks should convert the cart goal into a joint trajectory
    def goal_cb(self, msg):
        # MSG is a cartesian goal pose
        x0 = []
        if self.dmptype == "cart":
            x0 = np.array([self.arm_state.position.x, self.arm_state.position.y, self.arm_state.position.z, self.arm_state.orientation.x, self.arm_state.orientation.y, self.arm_state.orientation.z, self.arm_state.orientation.w])
        elif self.dmptype == "joint":
            if len(pose.position) == 7:
                #print msg
                x0 = np.array(self.arm_state.position)
            else:
                print "Issue with data length:",len(pose.position)
        traj = self.generate_traj(x0, msg) # result of IK
        self.traj[self.active_arm] = traj
        self.exec_traj = True
        print("Arm traj")

    #def right_pose_cb(self, msg):
    #    self.arm_pose["right"] = msg

    #def left_pose_cb(self, msg):
    #    self.arm_pose["left"] = msg

    def timer_cb(self, msg):
        if self.exec_traj:
            dims =[]
            print "Exec DMP"
            for d in self.traj[self.active_arm]:
                if self.dmptype == "cart":
                    cmd = Pose()
                    cmd.position.x = d[0]
                    cmd.position.y = d[1]
                    cmd.position.z = d[2]
                    cmd.orientation.x = d[3]
                    cmd.orientation.y = d[4]
                    cmd.orientation.z = d[5]
                    cmd.orientation.w = d[6]
                elif self.dmptype == "joint":
                    cmd = Float64MultiArray()
                    cmd.layout.dim.append(MultiArrayDimension())
                    cmd.layout.dim[0].label='RAD'
                    cmd.layout.dim[0].size=7
                    cmd.layout.dim[0].stride=0
                    cmd.layout.data_offset=0
                    cmd.data=d
                else:
                    print "Should never happen: type of DMP unknown, can't send command ; Exiting ..."
                    exit()

                begin = rospy.Time.now()
                timeout = 0.1
                minimal_walltime_timeout = 0.1
                try:
                    if self.active_arm == "left":
                        self.left_ptp_pub.publish(cmd)
                        # the minimal_walltime_timeout below sets the
                        # smallest wall-time timeout we handle.  if we
                        # try to wait for a smaller timeout it will be
                        # as much as this. A solution for simulation
                        # time, that allows for smaller
                        # walltime-timeouts (in case of a faster than
                        # realtime simulation time, still needs to be
                        # implemented.
                        while not self.left_ptp_done.wait(minimal_walltime_timeout):
                            if (rospy.Time.now() - begin) > rospy.Duration(timeout):
                                break
                            else:
                                pass
                        self.left_ptp_done.clear()
                    else:
                        self.right_ptp_pub.publish(cmd)
                        while not self.right_ptp_done.wait(minimal_walltime_timeout):
                            if (rospy.Time.now() - begin) > rospy.Duration(timeout):
                                break
                            else:
                                pass
                        self.right_ptp_done.clear()
                except Exception as e:
                    print e

            self.exec_traj = False
            signal_done = Bool()
            signal_done.data = True
            self.gesture_done_pub.publish(signal_done)
            print "Done"

if __name__ == "__main__":
    import argparse
    # Argument handling
    parser = argparse.ArgumentParser(description="DMP controller")
    #parser.add_argument('--demo', type=str, help="dmp demo data")
    #parser.add_argument('--demo', type=argparse.FileType('r'), nargs='+', help="dmp demo data")
    parser.add_argument('--demo', type=str, nargs='+', help="dmp demo data")
    #parser.add_argument('--robot', type=str, default="/simulation", help="simulation or real")
    parser.add_argument('--type', type=str, default="cart", help="cart or joint: cartesian or joint dmp")
    parser.add_argument('--save', action='store_true', help="should we save generated traj ?")
    parser.add_argument('--plot', action='store_true', help="should we display trajectories (demo, mean, generated) ?")
    args = parser.parse_args()
    rospy.init_node(args.type+"_dmp_ptp")
    print args.demo, args.type


    dcc = DmpController(db=args.demo, dmptype=args.type, saving=args.save, plotting=args.plot)

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
