#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
import matplotlib.pyplot as plt



class JointStatePlotter(Node):

    def __init__(self):
        super().__init__('joint_state_plotter')
        self.buffer_size = 50
        self.joint_positions_buffer = []
        self.time_buffer = []
        self.start_time = self.get_clock().now().nanoseconds / 1e9  # Convert to seconds
        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.lines = []
        plt.show()
    
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.listener_callback,
            10) 
        
        

    def listener_callback(self, msg):
        self.joint_positions_buffer.append(msg.position)
        self.time_buffer.append(self.get_clock().now().nanoseconds / 1e9 - self.start_time)  # Convert to seconds
        if len(self.joint_positions_buffer) > self.buffer_size:
            self.joint_positions_buffer.pop(0)
            self.time_buffer.pop(0)
        self.plot_joint_positions()
        
    def plot_joint_positions(self):
        self.ax.clear()
        if self.joint_positions_buffer:
            for i in range(len(self.joint_positions_buffer[0])):
                joint_data = [pos[i] for pos in self.joint_positions_buffer]
                self.ax.plot(self.time_buffer, joint_data, label=f'Joint {i+1}')
            self.ax.set_xlabel('Time')
            self.ax.set_ylabel('Position')
            self.ax.set_title('Joint Positions Over Time')
            self.ax.legend()
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()
        else:
            self.get_logger().warn('No joint positions to plot.')
        

def main(args=None):
    rclpy.init(args=args)

    joint_state_plotter = JointStatePlotter()

    rclpy.spin(joint_state_plotter)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    joint_state_plotter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()