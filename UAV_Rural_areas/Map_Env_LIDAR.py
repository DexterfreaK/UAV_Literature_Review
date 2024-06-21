import numpy as np
from map_disc import discretize_map  # Assuming this function exists in map_disc.py

class LidarMapper:
    def __init__(self, map_width, map_height, lidar_range):
        self.map_width = map_width
        self.map_height = map_height
        self.lidar_range = lidar_range
        self.center_x = map_width // 2
        self.center_y = map_height // 2
        
        # Initialize maps
        self.discretized_map = np.zeros((map_height, map_width), dtype=int)
        self.real_coor_map_x = np.zeros((map_height, map_width), dtype=float)
        self.real_coor_map_y = np.zeros((map_height, map_width), dtype=float)
        self.real_coor_map_z = np.zeros((map_height, map_width), dtype=float)
        
        # Set UAV position
        self.uav_flag = 99
        self.obs_flag = 1
        self.discretized_map[self.center_y, self.center_x] = self.uav_flag

    def get_lidar_data(self):
        # This function should be implemented to interface with your LIDAR sensor
        # It should return a list of (x, y, z) coordinates of detected obstacles
        pass

    def map_environment(self, uav_pos, threshold=0.1):
        x, y, z = uav_pos
        lidar_data = self.get_lidar_data()
        
        # Update UAV position in real coordinate maps
        self.real_coor_map_x[self.center_y, self.center_x] = x
        self.real_coor_map_y[self.center_y, self.center_x] = y
        self.real_coor_map_z[self.center_y, self.center_x] = z
        
        for obs_x, obs_y, obs_z in lidar_data:
            # Map LIDAR data to discretized map coordinates
            map_x = int(obs_x + self.center_x + 0.5)
            map_y = int(obs_y + self.center_y + 0.5)
            
            if 0 <= map_x < self.map_width and 0 <= map_y < self.map_height:
                # Update discretized map
                self.discretized_map[map_y, map_x] = self.obs_flag
                
                # Update real coordinate maps
                self.real_coor_map_x[map_y, map_x] = obs_x + x
                self.real_coor_map_y[map_y, map_x] = obs_y + y
                self.real_coor_map_z[map_y, map_x] = obs_z
        
        # Apply map discretization (assuming this function exists in map_disc.py)
        self.discretized_map = discretize_map(self.discretized_map)
        
        return self.discretized_map, (self.real_coor_map_x, self.real_coor_map_y, self.real_coor_map_z)

# Example usage
if __name__ == "__main__":
    mapper = LidarMapper(map_width=60, map_height=60, lidar_range=30)
    uav_position = (0, 0, 10)  # Example UAV position (x, y, z)
    
    discretized_map, real_coor_maps = mapper.map_environment(uav_position)
    
    print("Discretized Map:")
    print(discretized_map)
    print("\nReal Coordinate Maps (X, Y, Z):")
    print(real_coor_maps)