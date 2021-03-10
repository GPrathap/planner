import matplotlib.pyplot as plt
import rospy
import tf
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_matrix
import numpy as np
from matplotlib.animation import FuncAnimation
from matplotlib.lines import Line2D
import math
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker

class Visualiser:
    def __init__(self):
        plt.rcParams["font.size"] = "28"
        # self.fig = plt.figure()
        # self.ax1 = self.fig.add_subplot(3, 1, 1, sharex=True)
        # self.ax2 = self.fig.add_subplot(3, 1, 2)
        # self.ax3 = self.fig.add_subplot(3, 1, 3)
        self.fig, axs = plt.subplots(3, sharex=True)
        self.ax1 = axs[0]
        self.ax2 = axs[1]
        self.ax3 = axs[2]
        
        self.ln1 = Line2D([], [], color='red', linewidth=2, label=r'$p_x$')
        self.ln2 = Line2D([], [], color='red', linewidth=2, label=r'$p_y$')
        self.ln3 = Line2D([], [], color='red', linewidth=2, label=r'$p_z$')
        
        self.ln11 = Line2D([], [], color='blue', linewidth=2, label=r'$p^{ref}_x$')
        self.ln22 = Line2D([], [], color='blue', linewidth=2, label=r'$p^{ref}_y$')
        self.ln33 = Line2D([], [], color='blue', linewidth=2, label=r'$p^{ref}_z$')

        self.ax1.add_line(self.ln1)
        self.ax1.add_line(self.ln11)
        self.ax2.add_line(self.ln2)
        self.ax2.add_line(self.ln22)
        self.ax3.add_line(self.ln3)
        self.ax3.add_line(self.ln33)
         
        self.fig.text(0.06, 0.5, 'm', ha='center', va='center', rotation='vertical')

        self.x_data, self.y_data = [] , []
        self.odometry = None
        self.yaw_angles = []
        self.velocities = []
        self.velocities_com_x = []
        self.velocities_com_y = []
        self.velocities_com_z = []

        self.reference_trj = None
        self.real_trj = None
        reference = np.load("/home/geesara/catkin_ws/src/planner/Hagen-Planner/hagen_planner/state_machine/data/reference.npy")
        self.reference_trj = np.array(reference)[::2]
        self.reference_tmp = np.zeros((self.reference_trj.shape[0]+800, self.reference_trj.shape[1]))
        self.reference_tmp[800:] = self.reference_trj
        self.reference_trj = self.reference_tmp
        self.counter_p = 0
       
        
    def init_plot(self):
        velocity_=[-0.8, 0.8]
        self.ax1.set_xlim([0, 7500])
        custom_ticks = np.array([-5, 0, 9])
        self.ax1.set_yticks(custom_ticks)
        self.ax1.set_yticklabels(custom_ticks)
        ymin, ymax = self.ax1.get_ylim()
        self.ax1.vlines(x=[800, 6600], ymin=ymin, ymax=ymax, linewidth=6, color='g')
        legend = self.ax1.legend(loc='upper right', fontsize=28)
        
        self.ax2.set_xlim(0, 7500)
        custom_ticks = np.array([0, 6, 14])
        self.ax2.set_yticks(custom_ticks)
        self.ax2.set_yticklabels(custom_ticks)
        ymin, ymax = self.ax2.get_ylim()
        self.ax2.vlines(x=[800, 6600], ymin=ymin, ymax=ymax, linewidth=6, color='g')
        legend = self.ax2.legend(loc='upper right', fontsize=28)

        self.ax3.set_xlim(0, 7500)
        self.ax3.set_ylim(velocity_)
        custom_ticks = np.array([0, 2, 4])
        self.ax3.set_yticks(custom_ticks)
        self.ax3.set_yticklabels(custom_ticks)
        legend = self.ax3.legend(loc='upper right', fontsize=28)
        self.ax3.set_xlabel("number of readings (15 readings per second)")
        ymin, ymax = self.ax3.get_ylim()
        self.ax3.vlines(x=[800, 6600], ymin=ymin, ymax=ymax, linewidth=6, color='g')
        return [self.ln1, self.ln11, self.ln2, self.ln22, self.ln3, self.ln33]

    def getYaw(self, pose):
        quaternion = (pose.orientation.x, pose.orientation.y, pose.orientation.z,
                pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        yaw = euler[2] 
        return yaw
    
    def state_handler(self, msg):
        if(self.odometry is not None):
            yaw_angle = self.getYaw(self.odometry.pose.pose)
            
            velc = msg.twist.twist.linear 
            if(math.isinf(velc.x) or math.isinf(velc.y) or math.isinf(velc.z)):
                print("can not calcutate valusss")
            else:
                velocity = self.get_velocity(self.odometry)
                self.velocities_com_x.append(velc.x)
                self.velocities_com_y.append(velc.y)
                self.velocities_com_z.append(velc.z)
                self.velocities.append(velocity)
                self.yaw_angles.append(yaw_angle)
                x_index = len(self.x_data)
                self.x_data.append(x_index+1)
                # np.savetxt(self.save_data, np.array([velc.x, velc.y, velc.z, yaw_angle]), fmt='%1.3f', newline=",")
                # self.save_data.write("\n")

    def get_velocity(self, msg):
        velocity = msg.twist.twist.linear 
        return np.sqrt(velocity.x*2 + velocity.y*2 + velocity.z*2)  

    def reference_trajectory(self, msg):
        self.odometry = msg
        # self.state_handler(msg)
        # print("reference_trajectory: ", msg.id)
        points = np.array(msg.points)
        # if(msg.id == 300):
        #     if(points.shape[0]>0):
        #             self.reference_trj = self.process_data(points)
        #             print(self.reference_trj.shape)
        #             # np.savetxt(self.save_referance, self.reference_trj.transpose() , fmt='%1.3f', newline=",")
        #             # self.save_referance.write("\n")
        #             # np.save("reference.npy", self.reference_trj)
            
    def real_trajectory(self, msg):
        self.odometry = msg
        if(msg.id == 20):
            points = np.array(msg.points)
            if(points.shape[0]>0):
                self.real_trj = self.process_data(points)
                # np.savetxt(self.save_actual, self.real_trj.transpose() , fmt='%1.3f', newline=",")
                # self.save_actual.write("\n")
                # np.save("actual.npy", self.real_trj)
                # print("-----------")
                # print(self.real_trj.shape)
    # self.state_handler(msg)
    
    def process_data(self, points):
        prossed_data = []
        for i in points:
            prossed_data.append([i.x, i.y, i.z])
        return np.array(prossed_data)

    def update_plot(self, frame):
        if(self.real_trj is not None):
            self.x_data = np.arange(0, self.real_trj.shape[0], 1)
            self.ln1.set_data(self.x_data, self.real_trj[:,0])
            self.ln2.set_data(self.x_data, self.real_trj[:,1])
            self.ln3.set_data(self.x_data, self.real_trj[:,2])
            self.counter_p += 1
            # print(self.counter_p)
            if(self.reference_trj is not None):
                self.ln11.set_data(self.x_data, self.reference_trj[:,0][0:self.x_data.shape[0]])
                self.ln22.set_data(self.x_data, self.reference_trj[:,1][0:self.x_data.shape[0]])
                self.ln33.set_data(self.x_data, self.reference_trj[:,2][0:self.x_data.shape[0]])
        return [self.ln1, self.ln11, self.ln2, self.ln22, self.ln3, self.ln33]

rospy.init_node('lidar_visual_node')
vis = Visualiser()
sub = rospy.Subscriber('/planning_vis/trajectory', Marker, vis.reference_trajectory)
sub = rospy.Subscriber('/planning_vis/traj', Marker, vis.real_trajectory)

ani = FuncAnimation(vis.fig, vis.update_plot, init_func=vis.init_plot)
plt.show(block=True)