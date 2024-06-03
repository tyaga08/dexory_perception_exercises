import sys
import signal
from typing import List, Tuple
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, LaserScan
from nav_msgs.msg import Odometry
import numpy as np
import matplotlib.pyplot as plt
import sensor_msgs_py.point_cloud2 as pc2
import pprint


np.set_printoptions(threshold=sys.maxsize)


class SensorAnalysisNode(Node):
    def __init__(self):
        super().__init__("point_cloud_analysis_node")
        self.back_bpearl_sub = self.create_subscription(
            PointCloud2, "/lidars/bpearl_back_left", self.back_bpearl_callback, 10)
        self.front_bpearl_sub = self.create_subscription(
            PointCloud2, '/lidars/bpearl_front_right', self.front_bpearl_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.right_hokuyo_sub = self.create_subscription(
            LaserScan, '/lidars/hokuyo_back_right', self.right_hokuyo_callback, 10)
        self.left_hokuyo_sub = self.create_subscription(
            LaserScan, '/lidars/hokuyo_front_left', self.left_hokuyo_callback, 10)
        self.linear_twist = []
        self.angular_twist = []
        self.back_bpearl_pc2_max = []
        self.front_bpearl_pc2_max = []
        self.right_hokuyo_ls_max = []
        self.left_hokuyo_ls_max = []

        self._bpearl_theoretical_max_range = 90.0

    def front_bpearl_callback(self, msg: PointCloud2):
        self.front_bpearl_pc2_max += self._calc_bpearl_max_values(msg)

    def back_bpearl_callback(self, msg: PointCloud2):
        self.back_bpearl_pc2_max += self._calc_bpearl_max_values(msg)
    
    def right_hokuyo_callback(self, msg: LaserScan):
        self.right_hokuyo_ls_max += self._calc_hokuyo_max_values(msg)

    def left_hokuyo_callback(self, msg: LaserScan):
        self.left_hokuyo_ls_max += self._calc_hokuyo_max_values(msg)

    def odom_callback(self, msg: Odometry):
        lin_twist = msg.twist.twist.linear
        ang_twist = msg.twist.twist.angular
        self.linear_twist += [[lin_twist.x, lin_twist.y, lin_twist.z]]
        self.angular_twist += [[ang_twist.x, ang_twist.y, ang_twist.z]]

    def _calc_hokuyo_max_values(self, msg: LaserScan):
        ranges = np.array(msg.ranges, dtype=np.float32)
        intensities = np.array(msg.intensities, dtype=np.float32)
        arg_max = np.where(ranges >= 15)[0] # Checking for distances greater than 15m
        valid_int = np.where(intensities > 800.0)[0] # We don't want ranges with intensity less than 20% of max (4000 approx)
        reqd_ind = np.intersect1d(arg_max, valid_int)
        return [[ranges[i], intensities[i]] for i in reqd_ind]


    def _calc_bpearl_max_values(self, msg: PointCloud2):
        pc2_arr = pc2.read_points_numpy(msg, field_names=['x', 'y', 'z', 'intensity'], reshape_organized_cloud=True)
        nan_mask = np.isnan(pc2_arr).any(axis=2)
        pc2_arr = pc2_arr[~nan_mask]
        sq_distances = np.sum(np.square(pc2_arr[:, 0:3]), axis=1)
        arg_max = np.where(sq_distances >= 8100)[0] # checking for distances greater than 90m
        valid_int = np.where(pc2_arr[:,3] > 50.0)[0] # We don't want ranges with intensity less than 20% of max (250 approx)
        reqd_ind = np.intersect1d(arg_max, valid_int)
        return [[np.sqrt(sq_distances[i]), pc2_arr[i,3]] for i in reqd_ind]

    def _plot_speeds(self):
        if len(self.linear_twist) == 0:
            return
        self.linear_twist = np.array(self.linear_twist, dtype=np.float32)
        self.angular_twist = np.array(self.angular_twist, dtype=np.float32)
        fig, ax = plt.subplots(3,2)
        ax[0,0].plot(self.linear_twist[:,0])
        ax[0,0].set_title("Linear Twist: x")
        ax[1,0].plot(self.linear_twist[:,1])
        ax[1,0].set_title("Linear Twist: y")
        ax[2,0].plot(self.linear_twist[:,2])
        ax[2,0].set_title("Linear Twist: z")

        ax[0,1].plot(self.angular_twist[:,0])
        ax[0,1].set_title("Angular Twist: x")
        ax[1,1].plot(self.angular_twist[:,1])
        ax[1,1].set_title("Angular Twist: y")
        ax[2,1].plot(self.angular_twist[:,2])
        ax[2,1].set_title("Angular Twist: z")

        print("##################################")
        print("Min linear speed:", np.min(np.abs(self.linear_twist[:,0])))
        print("Max linear speed:", np.max(np.abs(self.linear_twist[:,0])))
        print("Min angular speed:", np.min(np.abs(self.angular_twist[:,2])))
        print("Max angular speed:", np.max(np.abs(self.angular_twist[:,2])))
        print("##################################")

    def _plot_bpearl_max_ranges(self, bpearl_vals: List[Tuple[np.float32, np.float32]], title: str = ''):
        if len(bpearl_vals) == 0:
            return
        bpearl_vals = np.array(bpearl_vals, dtype=np.float32)
        fig, ax = plt.subplots(1,1)
        ax.plot(bpearl_vals[:,0], label='Ranges above 90m')
        ax.plot(bpearl_vals[:,1], label='Intensities at these distance')
        ax.legend()
        ax.set_title(title)
    
    def _plot_hokuyo_max_ranges(self, ls_vals: List[Tuple[np.float32, np.float32]], title: str = ''):
        if len(ls_vals) == 0:
            return
        ls_vals = np.array(ls_vals, dtype=np.float32)
        fig, ax = plt.subplots(2,1)
        ax[0].plot(ls_vals[:,0], label='Ranges above 15m')
        ax[1].plot(ls_vals[:,1], label='Intensities at these distance')
        ax[0].legend()
        ax[1].legend()
        ax[0].set_title(title)
        ax[1].set_title(title)
    
    def post_process(self):
        print("\nPost processing")
        self._plot_speeds()
        self._plot_bpearl_max_ranges(self.back_bpearl_pc2_max, 'bpearl_back_left: intensities above 20% max values')
        self._plot_bpearl_max_ranges(self.front_bpearl_pc2_max, 'bpearl_front_right: intensities above 20% max values')
        self._plot_hokuyo_max_ranges(self.right_hokuyo_ls_max, 'hokuyo_back_right: intensities above 20% max values')
        self._plot_hokuyo_max_ranges(self.left_hokuyo_ls_max, 'hokuyo_front_left: intensities above 20% max values')
        plt.show()


def main(args=None):
    rclpy.init(args=args)
    node = SensorAnalysisNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.post_process()
        node.destroy_node()

if __name__ == '__main__':
    main()