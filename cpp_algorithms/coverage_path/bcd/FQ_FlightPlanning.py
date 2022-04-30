
import numpy as np
import matplotlib.pyplot as plt
import sys
import geopandas as gpd
#from cpp_algorithms.common_helpers import imshow, imshow_scatter
from bcd_helper import imshow, imshow_scatter
import math
from collections import defaultdict
from skimage.draw import polygon
import os
import random
from FQ_TaskAllocation import Decomposition, Auction, TrackDraw, Voronoi 
from FQ_GenTask import GetEFFNext, UpadteEFA, logEFA
from FQ_Drones_Info import Sensor, Drone


# we need know its flying height, sensor coverage! cover reward!!!!!!!!!!!
class FlightPlanner:
    def _init_(self, Drone, Tasks):
        pass
    def GenWaypoint(self):
        pass
    def Scheduling(self):
        pass

def PPM(I_H,FOV, h ):
    width=2*h*math.tan(FOV*math.pi/360)
    PPM=I_H/width
    print(f"With {width},PPM is {PPM}")
    return PPM

if __name__ == "__main__":
    
    
    
    
    
    Tasks=defaultdict(dict)  #  task: period, priority, 
    Tasks['f']=[5, 1]
    Tasks['a']=[10, 3]
    Tasks['BS']=[5, 1]
    Tasks['FT']=[1, 3]
    Tasks['FD']=[2, 3]
    Tasks['FI']=[3, 1]
    Tasks['FL']=[2, 2]
    ################ Get tasks.
    dir='/home/fangqiliu/eclipse-workspace_Python/Drone_path/CoveragePathPlanning-master/farsite/'
    foldername='FQ_burn'
    BunsiteBound=(701235.0,4308525.0,704355.0,4311675.0)
    BunsiteBound=(702374.0,4309425.0,703700.0,4310900.0)
    Bursite=(     702474.4,4309711.3,703511.1,4310784.9 )
    # igfile='/home/fangqiliu/eclipse-workspace_Python/Drone_path/CoveragePathPlanning-master/farsite/Rxfire/Burn_1/input/FQburn'
    # PutIginition(igfile,dir)
    init=120
    end=150
    Res=10
    EFA,EFAdict,Primdict =GetEFFNext(init,end,BunsiteBound,dir,foldername, Res=Res)
    #GetEFA("EFAdict.csv") 
    ########################## Do decomposition~ 
    DecomposeSize=200
    DecomposeSize=int((DecomposeSize/Res)*Res)
    Area,Fire=Decomposition(EFA, DecomposeSize, Res) # Need put tasks there!!!
    ########################## define drone as 
    DroneNum=6
    Drones=[]
    sensors=['RGB', 'THM', 'ALL', 'ALL','ALL','RGB']
    speeds=[5,5,5,5,5,5]
    FoV=[100,100,100, 80,80,100]
    for i in range(DroneNum):
        Drones.append(Drone(id=i,sensor=sensors[i], FoV=FoV[i],speed=speeds[i]))
    Drones,_=Auction(Area, Fire, Drones,EFA, Res,Tasks)
    #print(f"see Drones cost {[Drones[i].coverarea for i in range(len(Drones))]}")
    TrackDraw(Drones, EFA, Area, Fire)




































