#!/usr/bin/python
import math
from math import sin, cos, pi, radians
import rospy
import tf
from std_msgs.msg import Float32, Int32, Int16, Bool
import struct
from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
import numpy as np
DETECTION_DISTANCE = 0.6

def distanceToTiles(distance):
    n = round(distance/res)
    return int(n)

def obj_left(x0, y0, grid):
    x = distanceToTiles(DETECTION_DISTANCE)
    return raytrace(x0, y0, x, 0, grid)
def obj_right(x0, y0, grid):
    x = distanceToTiles(DETECTION_DISTANCE)
    return raytrace(x0, y0, -x, 0, grid)
def obj_front(x0, y0, grid):
    y = distanceToTiles(DETECTION_DISTANCE)
    return raytrace(x0, y0, 0, y, grid)
def obj_rear(x0, y0, grid):
    y = distanceToTiles(DETECTION_DISTANCE)
    return raytrace(x0, y0, 0, -y, grid)

def raytrace(x0, y0, x,y,grid):
    for i in range(min(x, x+x0), max(x, x+x0)):
        for j in range(min(y0+y, y), max(y0+y, y)):
            if grid[i,j] < 50 and grid[i,j] != -1:
                return True
    return False
def callbackCM(costmap):
    global cmap
    cmap = costmap
    m = grid2matrix(costmap)
    center_x = cmap.info.width/2 -1 
    center_y = cmap.info.height/2 -1
    if obj_left(center_x, center_y, m):
        obj_l.publish(Bool(True))
    if obj_right(center_x, center_y, m):
        obj_r.publish(Bool(True))
    if obj_front(center_x, center_y, m):
        obj_ff.publish(Bool(True))
    if obj_rear(center_x, center_y, m):
        obj_rr.publish(Bool(True))
def callbackUpdate(costmap_u):
    if cmap is None:
        print("waiting for initial costmap")
        return
    if costmap_u.x != 0 or costmap_u.y != 0:
        print("cannot handle costmap update")
        return
    global cmap
    cmap.data = costmap_u.data
    callbackCM(cmap)

def grid2matrix(occupancy_grid):
    global res
    res = occupancy_grid.info.resolution
    width = occupancy_grid.info.width
    height = occupancy_grid.info.height
    data = occupancy_grid.data
    matrix = []
    for x in range(0,width):
        matrix.append([])
        for y in range(0,height):
            mval = data[x*width + y]
            matrix[x].append(mval)
    return np.array(matrix)

global cmap
cmap=None
rospy.init_node('local_obj_detector')
obj_l = rospy.Publisher("/detection/left", Bool, queue_size=1)
obj_r = rospy.Publisher("/detection/right", Bool, queue_size=1)
obj_ff = rospy.Publisher("/detection/front", Bool, queue_size=1)
obj_rr = rospy.Publisher("/detection/rear", Bool, queue_size=1)
rospy.Subscriber("/move_base/local_costmap/costmap", OccupancyGrid, callbackCM, queue_size=5)
rospy.Subscriber("/move_base/local_costmap/costmap_updates", OccupancyGridUpdate, callbackUpdate, queue_size=5)
r = rospy.Rate(1)
rospy.spin()
