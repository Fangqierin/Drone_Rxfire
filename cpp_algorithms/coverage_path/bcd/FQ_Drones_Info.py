
import numpy as np
import matplotlib.pyplot as plt
import sys
import geopandas as gpd
import pandas as pd
#from cpp_algorithms.common_helpers import imshow, imshow_scatter
from bcd_helper import imshow, imshow_scatter
import math
from collections import defaultdict
import os
import random
from FQ_GenTask import GetEFFNext, UpadteEFA, logEFA
from geopandas._vectorized import length


class Sensor:
    def __init__(self, id=1, name=" ", type='ALL', FOV_H=72, FOV_V=58,I_H=3840, I_V=2160, size=4):
        self.id=id
        self.name=name
        self.type=type
        self.FOV_H=FOV_H
        self.FOV_V=FOV_V
        self.I_H=I_H
        self.I_V=I_V
        self.size=size
    def PPM_to_H(self,PPM,max_H, min_H):
        width=self.I_H/PPM
        
        h=width/(2**math.tan(self.FOV_H*math.pi/360))
        if h>max_H:
            h=100
        if h<min_H:
            print(f"Drones cannot get {PPM}")
            return 30
        length=2*h*math.tan(self.FOV_V*math.pi/360)
        width=2*h*math.tan(self.FOV_H*math.pi/360)
        FOV=min(length,width)
        print(f"Drones flies at {h} can get {PPM} with {int(FOV)}")
        return h
    # def PPM(I_H,FOV, h ):
    #     width=2*h*math.tan(FOV*math.pi/360)
    #     PPM=I_H/width
    #     print(f"With {width},PPM is {PPM}")
    #     return PPM
class Drone:
    def __init__(self, id=0,range=1000, speed=3, sensor=['ALL'], FoV=100): # sensor can be: sensor=['RGB', 'THM', 'All']
        self.id=id
        self.range=range
        self.sensortype=sensor
        self.speed=speed*60
        self.flytime=0
        self.coverarea=0
        self.area=[]
        self.FoV=FoV  # This should be computed by PPM!!!
        self.iniloc=[]
    

if __name__ == "__main__":
    # Tasks=defaultdict(dict)  #  task: period, priority, 
    # Tasks['f']=[5, 1]
    # Tasks['a']=[10, 3]
    # Tasks['BS']=[5, 1]
    # Tasks['FT']=[1, 3]
    # Tasks['FD']=[2, 3]
    # Tasks['FI']=[3, 1]
    # Tasks['FL']=[2, 2]
    ################ Get tasks.
    
    sensorfile='Data/sensor_info.csv'
    df=pd.read_csv(sensorfile,delimiter=r"\s+")
    print(len(df))
    for i in range(len(df)):
        ob=df.loc[i]
        s=Sensor(id=i, name=ob['name'], type=ob['type'], FOV_H=ob['FOV_H'], FOV_V=ob['FOV_V'],I_H=ob['I_H'], I_V=ob['I_V'], size=ob['size'])
        print(s.type=='RGB')
        if s.type=='RGB':
            s.PPM_to_H(25)
            s.PPM_to_H(65)
        else:
            print(f"see THERMAL")
            s.PPM_to_H(3.79)
            s.PPM_to_H(15.18)
    
    
    dir='/home/fangqiliu/eclipse-workspace_Python/Drone_path/CoveragePathPlanning-master/farsite/'
    foldername='FQ_burn'
    BunsiteBound=(701235.0,4308525.0,704355.0,4311675.0)
    BunsiteBound=(702374.0,4309425.0,703700.0,4310900.0)
    Bursite=( 702474.4,4309711.3,703511.1,4310784.9 )
    # igfile='/home/fangqiliu/eclipse-workspace_Python/Drone_path/CoveragePathPlanning-master/farsite/Rxfire/Burn_1/input/FQburn'
    # PutIginition(igfile,dir)
    init=120
    end=150
    Res=10
    EFA,EFAdict,Primdict =GetEFFNext(init,end,BunsiteBound,dir,foldername, Res=Res)
    #GetEFA("EFAdict.csv") 
    ########################## Do decomposition~ 
















