#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray

# Initialize the ROS node
rospy.init_node('uav_mapping_node')

def uav_mapping_callback(data):
    # Example data initialization, replace with actual data acquisition
    uavPos = np.array([0.0, 0.0, 0.0])
    lidData = np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]]) # Example LIDAR data
    mapWidth = 100
    mapHeight = 100
    obsInd = np.array([1, 1, 1]) # Example obstacle indicators
    mW = 10

    # Extract UAV position and LIDAR data
    x, y, z = uavPos
    lidX, lidY, lidZ = lidData[:, 0], lidData[:, 1], lidData[:, 2]
    obsIndX, obsIndY, obsIndZ = obsInd

    uavFlag = 99
    obsFlag = 1

    # Initialize the distance map and relative coordinate maps
    disMap = np.zeros((mapWidth, mapHeight), dtype=object)
    rCoMapX = np.zeros((mapWidth, mapHeight))
    rCoMapY = np.zeros((mapWidth, mapHeight))
    rCoMapZ = np.zeros((mapWidth, mapHeight))

    # Calculate center coordinates
    coCenterX = int(np.ceil(mapWidth / 2))
    coCenterY = int(np.ceil(mapHeight / 2))

    NDKIndex = coCenterX * mW + coCenterY
    disMap[coCenterX, coCenterY] = [uavFlag, z]

    rCoMapX[coCenterX, coCenterY] = x
    rCoMapY[coCenterX, coCenterY] = y
    rCoMapZ[coCenterX, coCenterY] = z

    # Process LIDAR data
    for i in range(len(lidData)):
        oX = int(np.ceil(lidX[i] + (mapWidth / 2)))
        oY = int(np.ceil(lidY[i] + (mapHeight / 2)))
        NRKIndex = oX * mW + oY

        disMap[oX, oY] = [obsFlag, obsIndZ]
        rCoMapX[oX, oY] = (lidX[i] + x) * obsIndX
        rCoMapY[oX, oY] = (lidY[i] + y) * obsIndY
        rCoMapZ[oX, oY] = lidZ[i] * obsIndZ

    rCoMaps = [rCoMapX, rCoMapY, rCoMapZ]

    # Print or return the maps for debugging
    rospy.loginfo(f"disMap: {disMap}")
    rospy.loginfo(f"rCoMaps: {rCoMaps}")

# Example subscriber to some topic that provides necessary data
rospy.Subscriber('uav_sensor_data', Float32MultiArray, uav_mapping_callback)

# Keep the node running
rospy.spin()
