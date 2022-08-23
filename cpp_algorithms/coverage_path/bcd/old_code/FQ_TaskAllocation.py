
import numpy as np
import matplotlib.pyplot as plt
import sys
import geopandas as gpd
from _collections import defaultdict
sys.path.append('../../../')
#from cpp_algorithms.common_helpers import imshow, imshow_scatter
from bcd_helper import imshow, imshow_scatter, cluster_imshow
from shapely.geometry import Point, Polygon
from pathlib import Path
# from zipfile import ZipFile
#from geopy import distance
#from skimage import measure
import math
from skimage.draw import polygon
import shapely
import shapely.geometry
from collections import defaultdict
from skimage.draw import polygon
import os
import random
from FQ_GenTask import GetEFFNext, UpadteEFA, logEFA,GenTasks
from FQ_Drones_Info import Sensor, Drone, ReadSen_PPM, LoadDrones

def Voronoi(EFA):    # decomposition using Voronoi method
    Grid={}
    Num=20
    p=np.full((len(EFA),len(EFA[0])), -1)
    # suppose 4 groups
    xn=random.sample(range(len(EFA)), Num)
    yn=random.sample(range(len(EFA[0])), Num)
    Group=defaultdict(list) 
    for i in range(Num):
        Group[i].append((xn[i],yn[i]))
    for i in range(len(EFA)):
        for j in range(len(EFA[i])):
            distances=[(i-xn[k])**2+(j-yn[k])**2 for k in range(Num)]
            C= distances.index(min(distances))
            p[i][j]=C
    #imshow(p)
    #plt.scatter(xn,yn, color='black')
    #plt.show()
    return p, xn,yn
def Decomposition(DecomposeSize, Res,Task_mission):  # Return the dictionary of grids and area
    DecomposeSize=int((DecomposeSize/Res)*Res)
    sc=DecomposeSize/Res
    AreaPara=defaultdict(dict)
    p=np.full((len(Task_mission['BM'][1]),len(Task_mission['BM'][1][0])), -1)
    cc=0
    #examples: Taskdict['FT']=[init, FTA,Missions['FT'][0],Missions['FT'][1]]
    tmp=list(Task_mission.keys())[0]
    perTable=np.full((len(Task_mission[tmp][1]),len(Task_mission[tmp][1][0])), 255)
    Task_mission=sorted(Task_mission.items(), key=lambda x:x[1][2])
    #print(f"Check {Task_mission}")
    #Here is tricky, I sort it by its period, so it grid will not overlapped
    #Every grid, we only consider the minimum period, right! 
    cc=0
    for mm in Task_mission:
        period=mm[1][2]
        cov=mm[1][1]
        Grid_mission=defaultdict(list)
        Area=defaultdict(list)
        x,y =np.where(cov==1)
        for i in range(len(x)):
            if perTable[x[i],y[i]]>period:
                ######### We will prioritize the short period! 
                perTable[x[i],y[i]]=period
                xc=int(x[i]/sc); yc=int(y[i]/sc)
                Area[(xc,yc)].append([x[i],y[i],mm[1][0]]) #initial time
                p[x[i],y[i]]=cc#xc+yc+cc*2
        AreaPara[mm[0]]=Area
        cc=cc+1
    # cluster_imshow(p,sc=sc)
    # plt.show()
    return AreaPara

def TrackDraw(Drones,BSshp,AreaPara):
    p=np.full(BSshp, -1)
    for i in range(len(Drones)):
        for g in Drones[i].area:
            mm=g[0]
            grids=AreaPara[mm].get(g[1])
            x=[k[0] for k in grids]
            y=[k[1] for k in grids]
            p[x,y]=i        
    imshow(p)
    plt.show()
    return p
    #plt.show()  
def HisLocDrone(Area, Bidders, GC):
    pass
def RanLocDrone(AreaPara, Bidders, sed):
    DNum=len(Bidders)
    grids=list(AreaPara['BM'].keys())
    # grids=list(Area.keys())
    # fires=list(Fire.keys())
    option=[k for k in grids]
    random.seed(sed)
    tmp=random.sample(range(len(option)), DNum)
    loc=[option[i] for i in tmp]
    return loc

class Bidder:
    def __init__(self, Drone): # sensor can be: sensor=['RGB', 'THM', 'All']
        self.id=Drone.id
        self.range=Drone.range
        self.sensortype=Drone.sensortype
        self.speed=Drone.speed
        self.FoVinfo=Drone.WPCinfo  # This should be computed by PPM!!!
        self.reward=defaultdict(dict)
        self.assignGrid=[]
        self.curcost=[]
        self.flytime=0
        self.coverarea=0
        self.area=[]
        self.iniArea=[]
    def GetCost(self, grid):
        if len(self.FoVinfo.get(grid[0]))==0:
            return 10000000000000
        FOV=max(list(self.FoVinfo.get(grid[0]).keys()))
        #return self.Gettravetime(grid)
        period=grid[-1]
        travel=self.Gettravetime(grid)
        if len(self.area)==0:
             return travel  ################ FQ, should we add initial location????????????????
        if travel>1 and len(self.area)>0:
            return 100000+self.coverarea+(grid[2]/(self.speed*FOV*period))
        if travel==0 and len(self.area)>0:
            return 0
        # if len(self.area)==0:
        #     return self.coverarea+(grid[2]/(self.speed*self.FoV*period))+travel
        return self.coverarea+(grid[2]/(self.speed*FOV*period))#+travel/self.speed
    def Gettravetime(self,grid):
        if len(self.assignGrid)==0:
            return math.sqrt((grid[1][0]-self.iniArea[0])**2+(grid[1][1]-self.iniArea[1])**2)
        add=min([math.sqrt((grid[1][0]-k[0])**2+(grid[1][1]-k[1])**2) for k in self.assignGrid])
        return add
        return self.flytime+(add/self.speed)
    def AddGrid(self, grid):
        self.area.append(grid)
        period=grid[-1]
        FOV=max(list(self.FoVinfo.get(grid[0]).keys()))
        self.coverarea=self.coverarea+(grid[2]/(self.speed*FOV*period))  # add area
        self.flytime= self.Gettravetime(grid)
        self.assignGrid.append(grid[1])
def Bidding(Grids, Bidders, AreaPara):
    while len(Grids)>0: 
        bid=[];costs=[]
        for i in range(len(Bidders)):
            costs.append([Bidders[i].GetCost(g) for g in Grids])
        bid=[min(k) for k in costs]
        price=min(bid)
        winners=[k for k in range(len(bid)) if bid[k]==price]
        rm=[]
        for j in winners:   # once have multiple winner or one winner????? 
            nums=[k for k in range(len(costs[j])) if costs[j][k]==price and Grids[k] not in rm]
            if len(nums)>0:
                Bidders[j].AddGrid(Grids[nums[0]])
                rm.append(Grids[nums[0]])
        for r in rm:
            Grids.remove(r)
    return Bidders
def Auction(AreaPara, Drones, Res,Missions, seed):
    Bidders=[]
    for d in Drones:
        Bidders.append(Bidder(Drone=d))
    Grids=[] # Get all grids
    for mm in AreaPara:
        period=Missions.get(mm)[0]
        for key in AreaPara[mm]:   #There is no overlapped area (from decomposition) 
            Grids.append([mm, key, len(AreaPara[mm][key])*(Res**2),AreaPara[mm][key],period])
    loc=RanLocDrone(AreaPara, Bidders,seed)
    for i in range(len(Bidders)):
        Bidders[i].iniArea=loc[i]
    Bidders=Bidding(Grids,Bidders,AreaPara)
    #########Fire first, then others
    # Drones=Bidding(FGrids,Drones,Area, Fire, EFA,Tasks)
    # Drones=Bidding(AGrids,Drones,Area, Fire, EFA,Tasks)
    for i in range(len(Bidders)):
        Drones[i].area=Bidders[i].area
        print(f"check what is area {Drones[i].area}")
    return Drones, Bidders

if __name__ == "__main__":
    Missions=defaultdict(dict)  #  Missions: id, period, priority, 
    Missions['BM']=[ 10, 1]  #Other area 
    Missions['FI']=[ 5, 1]  #track the fire 
    Missions['FT']=[ 3, 3]  # track the fire perimeter and the risky area (arrival within 10 min)
    Missions['FL']=[ 3, 2]  # FL is tracking the fire perimeter.
    Missions['FD']=[ 2, 3]
    ################ Get tasks.
    dir='/home/fangqiliu/eclipse-workspace_Python/Drone_path/CoveragePathPlanning-master/farsite'
    foldername='FQ_burn'
    # BunsiteBound=(701235.0,4308525.0,704355.0,4311675.0)
    # BunsiteBound=(702374.0,4309425.0,703700.0,4310900.0)
    Bursite=(702460.0,4309700.0,703540.0,4310820.0 )
    init=0; end=10; Res=10
    Task_mission,BSshp=GenTasks(init,end,Bursite,dir,foldername,Missions, Res=Res)
    ########################## Do decomposition~ 
    DecomposeSize=50
    AreaPara=Decomposition(DecomposeSize,Res,Task_mission) # Need put tasks there!!!
    print(f"check what is AreaPara {AreaPara}")
    ########################## Get drones
    sensorfile='Data/sensor_info.csv'
    PPMfile='Data/PPM_table.csv'
    DroneNum=3
    speeds=[5,5,5,5,5,5]
    loiter=[1,2,1,1,1]
    sensgrp=[['ZENMUSE_XT2_t','ZENMUSE_XT2_r'],['DJI_Air2S'],['ZENMUSE_XT2_t','ZENMUSE_XT2_r']]
    Drones=LoadDrones(sensorfile,PPMfile,DroneNum, speeds, sensgrp, Res,loiter)
    ##################################################### Task Allocation 
    Drones,Bidders=Auction(AreaPara, Drones, Res,Missions,1)
    print(f"see Drones cost {[Bidders[i].coverarea for i in range(len(Drones))]}")
    TrackDraw(Drones, BSshp, AreaPara)




















