# written by Harshit Nagpal

#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import Point

# Helper function to find direction
def find_direction(currCoor):
    # Example implementation, needs to be customized based on your use case
    return [True, False, True]  # Dummy direction list

# Helper function to calculate Euclidean distance
def euclidean_distance(p1, p2):
    return np.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2 + (p1.z - p2.z)**2)

# Helper function to check for obstacles
def check_obs(parGoal):
    # Dummy check for obstacles
    # Implement your obstacle checking mechanism here
    return 1  # Return 1 if no obstacle, 0 if obstacle

def estimate_partial_goal(currCoor, goalCoor, topPrevStack, dirPattern):
    direction = find_direction(currCoor)
    parGoal = [None] * len(direction)
    
    for i in range(len(direction)):
        dis = euclidean_distance(topPrevStack, goalCoor)
        if direction[i]:
            parGoal[i] = Point()
            parGoal[i].x = currCoor.x + dirPattern[i].x + dis
            parGoal[i].y = currCoor.y + dirPattern[i].y + dis
            parGoal[i].z = currCoor.z + dirPattern[i].z + dis

            check = check_obs(parGoal[i])
            parGoal[i].x *= check
            parGoal[i].y *= check
            parGoal[i].z *= check

            parGoal[i] = euclidean_distance(parGoal[i], goalCoor)

    selParGoal = min(parGoal)
    return selParGoal

def main():
    rospy.init_node('partial_goal_estimator')

    currCoor = Point(1.0, 2.0, 0.0)
    goalCoor = Point(5.0, 5.0, 0.0)
    topPrevStack = Point(0.0, 0.0, 0.0)
    dirPattern = [Point(1,0,0), Point(0,1,0), Point(-1,0,0)]  # Example direction patterns

    selParGoal = estimate_partial_goal(currCoor, goalCoor, topPrevStack, dirPattern)
    
    rospy.loginfo("Selected Partial Goal: %s", selParGoal)

if __name__ == '__main__':
    main()
