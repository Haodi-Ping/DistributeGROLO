import numpy as np
from initPos import *
# def DV_distance(robots, distanceList):

def setInitial(robots):
    # use dv_distance to calculate the initial position
    dv_distance()
    dv_list = np.loadtxt('data/dv_distance.npy')
    for index in range(len(dv_list)):
        robots[index].set_coord(dv_list[index][0], dv_list[index][1])
        print('robot ', index, dv_list[index])
