
import numpy as np
import matplotlib.pyplot as plt
import sys
import geopandas as gpd
sys.path.append('../../../')
#from cpp_algorithms.common_helpers import imshow, imshow_scatter
from bcd_helper import imshow, imshow_scatter
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
from FQ_GenTask import GetEFFNext, UpadteEFA, logEFA
from FQ_Drones_Info import Sensor, Drone

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
    imshow(p)
    #plt.scatter(xn,yn, color='black')
    plt.show()
    return p, xn,yn


def Decomposition(EFA, DecomposeSize, Res):  # Return the dictionary of 
    sc=DecomposeSize/Res
    p=np.full((len(EFA),len(EFA[0])), -1)
    Area=defaultdict(list)
    Fire=defaultdict(list)
    for i in range(len(EFA)):
        for j in range(len(EFA[i])):
            xc=int(i/sc); yc=int(j/sc)
            if EFA[i,j]==255:
                Area[(xc,yc)].append([i,j])
                p[i,j]=xc**2+yc**2
            else:
                Fire[(xc,yc)].append([i,j])
                p[i,j]=-(xc+yc)
    # imshow(p)
    # plt.show()
    return Area, Fire
def TrackDraw(Drones, EFA, Area, Fire):
    p=np.full((len(EFA),len(EFA[0])), -1)
    for i in range(len(Drones)):
        # grids=Area.get(Drones[i].iniloc)
        # x=[k[0] for k in grids]
        # y=[k[1] for k in grids]
        # p[x,y]=i
        for a in Drones[i].area:
            if a[0]=='a':
                grids=Area.get(a[1])
            else:
                grids=Fire.get(a[1])
            x=[k[0] for k in grids]
            y=[k[1] for k in grids]
            p[x,y]=i        
    imshow(p)
    #plt.legend(handles=patches, loc=4, borderaxespad=0.)
    plt.show()  
    
def HisLocDrone(Area, Bidders, GC):
    pass
def RanLocDrone(Area, Fire, Bidders, sed):
    DNum=len(Bidders)
    grids=list(Area.keys())
    fires=list(Fire.keys())
    option=[k for k in grids if k not in fires]
    random.seed(sed)
    tmp=random.sample(range(len(option)), DNum)
    loc=[option[i] for i in tmp]
    return loc

class Bidder:
    def __init__(self, Drone): # sensor can be: sensor=['RGB', 'THM', 'All']
        self.id=Drone.id
        self.range=Drone.range
        self.sensor=Drone.sensortype
        self.speed=Drone.speed
        self.FoV=Drone.FoV  # This should be computed by PPM!!!
        self.reward=defaultdict(dict)
        self.assignGrid=[]
        self.curcost=[]
        self.flytime=0
        self.coverarea=0
        self.area=[]
        self.iniArea=[]
    def GetCost(self, grid):
        if self.sensor=='RGB':
            if grid[0]=='f':
                return 1000000000
        #return self.Gettravetime(grid)
        period=grid[-1]
        travel=self.Gettravetime(grid)
        if len(self.area)==0:
             return travel  ################ FQ, should we add initial location????????????????
        if travel>1 and len(self.area)>0:
            return 100000+self.coverarea+(grid[2]/(self.speed*self.FoV*period))
        if travel==0 and len(self.area)>0:
            return 0
        # if len(self.area)==0:
        #     return self.coverarea+(grid[2]/(self.speed*self.FoV*period))+travel
        return self.coverarea+(grid[2]/(self.speed*self.FoV*period))#+travel/self.speed
        #return self.coverarea+(grid[2]/(self.speed*self.FoV*period))+self.Gettravetime(grid)
    def Gettravetime(self,grid):
        if len(self.assignGrid)==0:
            #print(f"see iniArea {self.iniArea}")
            return math.sqrt((grid[1][0]-self.iniArea[0])**2+(grid[1][1]-self.iniArea[1])**2)
        add=min([math.sqrt((grid[1][0]-k[0])**2+(grid[1][1]-k[1])**2) for k in self.assignGrid])
        #return add/self.speed
        return add
        return self.flytime+(add/self.speed)
    def AddGrid(self, grid):
        self.area.append(grid)
        period=grid[-1]
        self.coverarea=self.coverarea+(grid[2]/(self.speed*self.FoV*period))  # add area
        self.flytime= self.Gettravetime(grid)
        self.assignGrid.append(grid[1])
def Bidding(Grids, Bidders, Area, Fire, EFA,Tasks):
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
                #print(f" Drone {j} add {Grids[nums[0]]}")
                # if Grids[nums[0]][0]=='f':
                #     tmp=Grids[nums[0]][1]
                #     areagrid=['a', tmp, len(Area[tmp])*(Res**2),Tasks.get('a')[0]]
                #     Bidders[j].AddGrid(areagrid)
                #     print(f"we also assign area {areagrid}")
                #     rm.append(areagrid)
                rm.append(Grids[nums[0]])
            #else:
                #print(f" Drone {j} cannot get {Grids[nums[0]]}")
        for r in rm:
            Grids.remove(r)
        #TrackDraw(Bidders, EFA, Area, Fire)
    return Bidders
def Auction(Area, Fire, Drones, EFA, Res,Tasks):
    Bidders=[]
    for d in Drones:
        Bidders.append(Bidder(Drone=d))
    Grids=[] # Get all grids
    FGrids=[]; AGrids=[]
    for key in list(Fire.keys()):
        period=Tasks.get('f')[0]
        #print(f"check f",key,len(Fire[key])*(Res**2),Fire[key])
        Grids.append(['f', key, len(Fire[key])*(Res**2),period])
        FGrids.append(['f', key, len(Fire[key])*(Res**2),period])
    for key in list(Area.keys()):
        #print(f"check",key,len(Area[key]),len(Area[key])*(Res**2),Area[key])
        period=Tasks.get('a')[0]  ################################################################ Should change to task!!!!
        Grids.append(['a', key, len(Area[key])*(Res**2), period]) # Get the real area 
        AGrids.append(['a', key, len(Area[key])*(Res**2),period])
    loc=RanLocDrone(Area,Fire, Bidders,0)
    for i in range(len(Bidders)):
        Bidders[i].iniArea=loc[i]
    Bidders=Bidding(Grids,Bidders,Area, Fire, EFA,Tasks)
    #########Fire first, then others
    # Drones=Bidding(FGrids,Drones,Area, Fire, EFA,Tasks)
    # Drones=Bidding(AGrids,Drones,Area, Fire, EFA,Tasks)
    for i in range(len(Bidders)):
        Drones[i].area=Bidders[i].area
    
    return Drones, Bidders

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
    Area,Fire=Decomposition(EFA, DecomposeSize,Res) # Need put tasks there!!!
    ########################## define drone as 
    DroneNum=6
    Bidders=[]
    Drones=[]
    sensortype=['RGB', 'THM', 'ALL', 'ALL','ALL','RGB']
    speeds=[5,5,5,5,5,5]
    FoV=[100,100,100, 80,80,100]
    for i in range(DroneNum):
        Drones.append(Drone(id=i,sensor=sensortype[i], FoV=FoV[i],speed=speeds[i]))
    Drones,Bidders=Auction(Area, Fire, Drones,EFA, Res,Tasks)
    print(f"see Drones cost {[Bidders[i].coverarea for i in range(len(Drones))]}")
    TrackDraw(Drones, EFA, Area, Fire)

















