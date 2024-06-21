#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

class UAVMapping:
    def __init__(self):
        rospy.init_node('uav_mapping_node')
        
        # Parameters
        self.map_width = rospy.get_param('~map_width', 100)
        self.map_height = rospy.get_param('~map_height', 100)
        self.lidar_range = rospy.get_param('~lidar_range', 50)
        
        # Initialize maps
        self.discretized_map = np.zeros((self.map_width, self.map_height), dtype=int)
        self.real_coor_map_x = np.zeros((self.map_width, self.map_height))
        self.real_coor_map_y = np.zeros((self.map_width, self.map_height))
        self.real_coor_map_z = np.zeros((self.map_width, self.map_height))
        
        # ROS subscribers
        rospy.Subscriber('/uav/pose', PoseStamped, self.uav_pose_callback)
        rospy.Subscriber('/uav/lidar', PointCloud2, self.lidar_callback)
        
        self.uav_pos = [0, 0, 0]
        
    def uav_pose_callback(self, msg):
        self.uav_pos = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        
    def lidar_callback(self, point_cloud):
        x, y, z = self.uav_pos
        
        center_x = self.map_width // 2
        center_y = self.map_height // 2
        
        # Set UAV position in discretized map
        self.discretized_map[center_x, center_y] = 99
        self.real_coor_map_x[center_x, center_y] = x
        self.real_coor_map_y[center_x, center_y] = y
        self.real_coor_map_z[center_x, center_y] = z
        
        # Process LIDAR data
        for point in pc2.read_points(point_cloud, field_names=("x", "y", "z"), skip_nans=True):
            px, py, pz = point
            
            # Convert to map coordinates
            map_x = int(px + center_x)
            map_y = int(py + center_y)
            
            if 0 <= map_x < self.map_width and 0 <= map_y < self.map_height:
                self.discretized_map[map_x, map_y] = 1  # Obstacle
                self.real_coor_map_x[map_x, map_y] = x + px
                self.real_coor_map_y[map_x, map_y] = y + py
                self.real_coor_map_z[map_x, map_y] = pz
        
        # Publish or use the updated maps as needed
        self.publish_maps()
        
    def publish_maps(self):
        # Implement method to publish the updated maps
        # This could involve converting the numpy arrays to ROS messages
        # and publishing them on appropriate topics
        pass

if __name__ == '__main__':
    try:
        mapping = UAVMapping()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass