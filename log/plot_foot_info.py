#!/usr/bin/env python

import rospy
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from sensor_msgs.msg import JointState

class JointStateVisualizer:
    def __init__(self):
        # init ros
        rospy.init_node('joint_state_visualizer', anonymous=True)
        self.sub = rospy.Subscriber('/g1/joint_state', JointState, self.callback)
        
        # data buffer
        self.data_buffer_size = 10000
        self.time_buffer = []
        self.left_pos_x = []
        self.left_pos_y = []
        self.left_pos_z = []
        self.right_pos_x = []
        self.right_pos_y = []
        self.right_pos_z = []
        
        self.left_vel_x = []
        self.left_vel_y = []
        self.left_vel_z = []
        self.right_vel_x = []
        self.right_vel_y = []
        self.right_vel_z = []
        
        # init
        self.fig, self.axs = plt.subplots(2, 2, figsize=(9, 6))
        self.axs[0, 0].set_title('Left pos (x, y, z)')
        self.axs[0, 1].set_title('Righ pos (x, y, z)')
        self.axs[1, 0].set_title('Left vel (x, y, z)')
        self.axs[1, 1].set_title('Righ vel (x, y, z)')
        
        for ax in self.axs.flat:
            ax.set_xlabel('time')
            ax.set_ylabel('value')
            ax.grid(True)

        self.left_pos_lines = [self.axs[0, 0].plot([], [], label=f'pos {axis}')[0] for axis in ['x', 'y', 'z']]
        self.right_pos_lines = [self.axs[0, 1].plot([], [], label=f'pos {axis}')[0] for axis in ['x', 'y', 'z']]
        self.left_vel_lines = [self.axs[1, 0].plot([], [], label=f'vel {axis}')[0] for axis in ['x', 'y', 'z']]
        self.right_vel_lines = [self.axs[1, 1].plot([], [], label=f'vel {axis}')[0] for axis in ['x', 'y', 'z']]
        
        self.axs[0, 0].legend()
        self.axs[0, 1].legend()
        self.axs[1, 0].legend()
        self.axs[1, 1].legend()
        
        self.anim = FuncAnimation(self.fig, self.update_plot, interval=50, blit=False)
        
        plt.tight_layout()
        plt.subplots_adjust(top=0.9)
        plt.show()
    
    def callback(self, msg):
        if not hasattr(msg, 'position') or not hasattr(msg, 'velocity'):
            return
        
        current_time = rospy.Time.now().to_sec()
        
        # get position data (first 3d are left foot position, last 3d are right foot position)
        left_pos = msg.position[:3]
        right_pos = msg.position[3:]
        
        # get velocity data (first 3d are left foot velocity, last 3d are right foot velocity)
        left_vel = msg.velocity[:3]
        right_vel = msg.velocity[3:]

        self.time_buffer.append(current_time)
        self.left_pos_x.append(left_pos[0])
        self.left_pos_y.append(left_pos[1])
        self.left_pos_z.append(left_pos[2])
        self.right_pos_x.append(right_pos[0])
        self.right_pos_y.append(right_pos[1])
        self.right_pos_z.append(right_pos[2])
        
        self.left_vel_x.append(left_vel[0])
        self.left_vel_y.append(left_vel[1])
        self.left_vel_z.append(left_vel[2])
        self.right_vel_x.append(right_vel[0])
        self.right_vel_y.append(right_vel[1])
        self.right_vel_z.append(right_vel[2])
        
        if len(self.time_buffer) > self.data_buffer_size:
            self.time_buffer.pop(0)
            self.left_pos_x.pop(0)
            self.left_pos_y.pop(0)
            self.left_pos_z.pop(0)
            self.right_pos_x.pop(0)
            self.right_pos_y.pop(0)
            self.right_pos_z.pop(0)
            
            self.left_vel_x.pop(0)
            self.left_vel_y.pop(0)
            self.left_vel_z.pop(0)
            self.right_vel_x.pop(0)
            self.right_vel_y.pop(0)
            self.right_vel_z.pop(0)
    
    def update_plot(self, frame):

        if not self.time_buffer:
            return []
        
        # update left foot position curve
        self.left_pos_lines[0].set_data(self.time_buffer, self.left_pos_x)
        self.left_pos_lines[1].set_data(self.time_buffer, self.left_pos_y)
        self.left_pos_lines[2].set_data(self.time_buffer, self.left_pos_z)
        
        # update right foot position curve
        self.right_pos_lines[0].set_data(self.time_buffer, self.right_pos_x)
        self.right_pos_lines[1].set_data(self.time_buffer, self.right_pos_y)
        self.right_pos_lines[2].set_data(self.time_buffer, self.right_pos_z)
        
        # update left foot velocity curve
        self.left_vel_lines[0].set_data(self.time_buffer, self.left_vel_x)
        self.left_vel_lines[1].set_data(self.time_buffer, self.left_vel_y)
        self.left_vel_lines[2].set_data(self.time_buffer, self.left_vel_z)
        
        # update right foot velocity curve
        self.right_vel_lines[0].set_data(self.time_buffer, self.right_vel_x)
        self.right_vel_lines[1].set_data(self.time_buffer, self.right_vel_y)
        self.right_vel_lines[2].set_data(self.time_buffer, self.right_vel_z)
        
        for ax in self.axs.flat:
            ax.relim()
            ax.autoscale_view()
        
        return self.left_pos_lines + self.right_pos_lines + self.left_vel_lines + self.right_vel_lines

if __name__ == '__main__':
    try:
        visualizer = JointStateVisualizer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
