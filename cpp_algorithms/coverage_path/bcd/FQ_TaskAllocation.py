
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
def get_hv_wh(BunsiteBound):
    """
    Get haversine calcualted width and height of
    the smallest bounding rectangle of the coverage area.
    """
    llng, llat, rlng, rlat =  BunsiteBound
    w=abs(llng-rlng)
    h=abs(llat-rlat)
    print(f"FQ Weight: {w}; Height: {h}")
    return w, h
"""
Convert to raster
"""
def get_scale(BunsiteBound, meter=1):
    """
#     Returns the supposed longside of the area.
#     """
    w,h = get_hv_wh(BunsiteBound)
    return int(np.round((np.array([w,h])/meter).max()))
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
# def imshow(area_map,r=1,c=1,i=1,figsize=(5,5),cmap="viridis"):
#     """
#     Display with no interpolation.
#     """
#     if r < 2 and c < 2 or i == 1:
#         plt.figure(figsize=figsize)
#     plt.subplot(r,c,i)
#     #ax = plt.imshow(area_map,interpolation='none',cmap=cmap)
#     ax = plt.imshow(area_map.T,interpolation='none',cmap=cmap,origin='lower')
#     plt.axis('off');
#     return ax

def get_raster(gpdf_final, scale=2000, BunsiteBound=0, NF=-1, F=1, P=2, Res=1):
    """
    Returns rasterised version for the projection
    """
    #Res=30
    mn=np.array(BunsiteBound[:2])
    sh=(np.array(BunsiteBound[2:])-mn)# FQ change the boundary
    sh=np.int64(sh/Res)
    p = np.full(sh,NF) #FQ change that
    empty=np.full(sh,NF) #for record perimeter
    #prim=[[]for i in range(2)]
    for i in range(len(gpdf_final)):
        shp=gpdf_final.loc[i, 'geometry']
        ext = np.array(shp.exterior)
        mix = (ext - mn)
        mx=(np.array(BunsiteBound[2:])-mn).max()  #FQ changed that
        mix1=np.ceil(mix/Res)#*3
        mix1 = np.int64(mix1)
        r,c = polygon(*mix1.T,sh)   #fq? should reverse that? 
        p[r,c] = F
        r,c=mix1.T # for the fire perimeter 
        #print(r,c)
        empty[r,c]=0
        p[r,c]=P
        mix2=np.floor(mix/Res)#*30
        mix2=np.int64(mix2)
        r,c = polygon(*mix2.T,sh)
        p[r,c] = F
        r,c = mix2.T
        p[r,c] = P 
        empty[r, c]=0
    x,y=np.where(empty==0)
    #imshow(empty)
    # imshow(p)
    # plt.show()
    #print(f"FQ change to  {len(x)}")
    return p,[x,y]
def Firefront(maps):
    cu_map=maps[0]
    fu_map=maps[1]
    return cu_map-fu_map

def draw(area_map_):
    img=area_map_
    matrix = np.full(img.shape, 0, dtype=np.uint8)
    matrix[img == -1] = 255
    imshow(matrix, figsize=(8, 8), cmap="cividis")   
    plt.show(block=True)

def logEFA(EFAdict,Primdict, filename, shape, Res):
    f = open(filename, "w") 
    print(shape[0])
    f.write(f"{shape[0]} {shape[1]}\n ")
    for key in EFAdict.keys():
        row=np.array(EFAdict[key])
        f.write(f"{key} {len(list(row[0]))} ")
        for i in list(row[0]):
            f.write(f"{i},")
        f.write(f" ")
        for i in list(row[1]):
            f.write(f"{i},")
        f.write(f"\n")
        f.write(f"Prim {len(list(Primdict.get(key)[0]))} ")
        for j in list(Primdict.get(key)[0]):
            f.write(f"{j},")
        for j in list(Primdict.get(key)[1]):
            f.write(f"{j},")
        f.write(f"\n")
        #print(f"{key} {list(row[0])} {list(row[1])}")
    f.close() 
def GetEFA(filename):
    lines = []
    with open(filename) as f:
        lines = f.readlines()  
    #print(len(lines)) 
    # for line in lines: 
    #     print(f"FQ {line}")

def PutIginition(filename,dir):
    tofile=f"{dir}_0_Perimeters"
    c1=f"cp {filename}.shp {tofile}.shp"
    c2=f"cp {filename}.shx {tofile}.shx"
    os.system(c1)
    os.system(c2)
    print(f"Run {c1}")

def UpadteEFA(Pmap, area_map,time,EFA, NF=-1, F=1, P=2): #    Get the estimated fire arrival time.
    if len(Pmap)==1:
        img=area_map
        EFA = np.full(img.shape, -1,dtype=np.uint8)
        EFA[img == F] = time
        EFA[img == P] = time
        x,y= np.where(img==0)
    else:
        tmp=area_map-Pmap     # Get first arrival time. 
        #imshow(area_map)
        EFA[tmp>0]=time
        #imshow(EFA)
        #plt.show()
        x,y=np.where(EFA==time)
    return EFA, x,y
  
def GetEFFNext(init,end, BunsiteBound, dir, foldername, Res=10):
    scale = get_scale(BunsiteBound, meter=1)
    EFAdict=defaultdict(list)
    timestamp=3
    time=init
    Primdict=defaultdict(list)
    Pmap=[0]
    EFA=[]  # EFA is matrix shows the expected fire arrival time
    while time<=end:
        file=f"{dir}/{foldername}/output/{foldername}_{time}_Perimeters.shp"
        data=gpd.read_file(file)
        area_map_,prim= get_raster(data, scale, BunsiteBound, NF=-1, F=1, P=2, Res=Res)
        EFA,x,y=UpadteEFA(Pmap, area_map_,time,EFA)
        EFAdict[time]=[x,y]
        Primdict[time]=prim
        Pmap=area_map_ 
        time=time+timestamp
    #shape=area_map_.shape
    #logEFA(EFAdict, Primdict,"EFAdict.csv",shape, 10)
    return EFA, EFAdict, Primdict

def Decomposition(EFA, DecomposeSize):  # Return the dictionary of 
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
    
def HisLocDrone(Area, Drones, GC):
    pass
def RanLocDrone(Area, Fire, Drones, sed):
    DNum=len(Drones)
    grids=list(Area.keys())
    fires=list(Fire.keys())
    option=[k for k in grids if k not in fires]
    random.seed(sed)
    tmp=random.sample(range(len(option)), DNum)
    loc=[option[i] for i in tmp]
    return loc
class Sensor:
    def _init_(self, Type='ALL', Pixel=1080, AFoV=math.pi*(60/180)):
        pass
class Drone:
    def __init__(self, id=0,range=1000, speed=3, sensor=['ALL'], FoV=100): # sensor can be: sensor=['RGB', 'THM', 'All']
        self.id=id
        self.range=range
        self.sensor=sensor
        self.speed=speed*60
        self.reward=defaultdict(dict)
        self.assignGrid=[]
        self.curcost=[]
        self.flytime=0
        self.coverarea=0
        self.area=[]
        self.FoV=FoV  # This should be computed by PPM!!!
        self.iniloc=[]
    def GetCost(self, grid):
        if self.sensor=='RGB':
            if grid[0]=='f':
                return 1000000000
        #return self.Gettravetime(grid)
        period=grid[-1]
        #print(f"why period {period}")
        #print(f"see cost", grid[2], self.speed*self.FoV*period)
        travel=self.Gettravetime(grid)
        print(f" Drone {self.id} travel to {grid} {travel}")
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
            return math.sqrt((grid[1][0]-self.iniloc[0])**2+(grid[1][1]-self.iniloc[1])**2)
        add=min([math.sqrt((grid[1][0]-k[0])**2+(grid[1][1]-k[1])**2) for k in self.assignGrid])
        #print(f"why {grid[1]}, {add} {self.assignGrid}")
        #return add/self.speed
        return add
        return self.flytime+(add/self.speed)
    def AddGrid(self, grid):
        self.area.append(grid)
        period=grid[-1]
        self.coverarea=self.coverarea+(grid[2]/(self.speed*self.FoV*period))  # add area
        self.flytime= self.Gettravetime(grid)
        self.assignGrid.append(grid[1])
        #print(f"see the flytime is {self.flytime}") 
def Bidding(Grids, Drones, Area, Fire, EFA,Tasks):
    while len(Grids)>0: 
        bid=[];costs=[]
        for i in range(len(Drones)):
            costs.append([Drones[i].GetCost(g) for g in Grids])
        bid=[min(k) for k in costs]
        print(f"see the bid {bid}")
        price=min(bid)
        winners=[k for k in range(len(bid)) if bid[k]==price]
        print(f"see winners {winners}")
        rm=[]
        for j in winners:   # once have multiple winner or one winner????? 
        #for j in [winners[0]]:   # single winner or multiple winner is the same!!!!
            #num=costs[j].index(price)
            nums=[k for k in range(len(costs[j])) if costs[j][k]==price and Grids[k] not in rm]
            print(f"Drone {j} get {nums}")
            #while Grids[nums[n]] in rm:
            if len(nums)>0:
                Drones[j].AddGrid(Grids[nums[0]])
                print(f" Drone {j} add {Grids[nums[0]]}")
                # if Grids[nums[0]][0]=='f':
                #     tmp=Grids[nums[0]][1]
                #     areagrid=['a', tmp, len(Area[tmp])*(Res**2),Tasks.get('a')[0]]
                #     Drones[j].AddGrid(areagrid)
                #     print(f"we also assign area {areagrid}")
                #     rm.append(areagrid)
                rm.append(Grids[nums[0]])
            #else:
                #print(f" Drone {j} cannot get {Grids[nums[0]]}")
        for r in rm:
            Grids.remove(r)
        #TrackDraw(Drones, EFA, Area, Fire)
    return Drones
def Auction(Area, Fire, Drones, EFA, Res,Tasks):
    Grids=[] # Get all grids
    FGrids=[]; AGrids=[]
    for key in list(Fire.keys()):
        period=Tasks.get('f')[0]
        print(f"check f",key,len(Fire[key])*(Res**2),Fire[key])
        Grids.append(['f', key, len(Fire[key])*(Res**2),period])
        FGrids.append(['f', key, len(Fire[key])*(Res**2),period])
    for key in list(Area.keys()):
        print(f"check",key,len(Area[key]),len(Area[key])*(Res**2),Area[key])
        period=Tasks.get('a')[0]  ################################################################ Should change to task!!!!
        Grids.append(['a', key, len(Area[key])*(Res**2), period]) # Get the real area 
        AGrids.append(['a', key, len(Area[key])*(Res**2),period])
    loc=RanLocDrone(Area,Fire, Drones,0)
    for i in range(len(Drones)):
        Drones[i].iniloc=loc[i]
    Drones=Bidding(Grids,Drones,Area, Fire, EFA,Tasks)
    #########Fire first, then others
    # Drones=Bidding(FGrids,Drones,Area, Fire, EFA,Tasks)
    # Drones=Bidding(AGrids,Drones,Area, Fire, EFA,Tasks)
    return Drones


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
Area,Fire=Decomposition(EFA, DecomposeSize) # Need put tasks there!!!
########################## define drone as 
DroneNum=6
Drones=[]
sensors=['RGB', 'THM', 'ALL', 'ALL','ALL','RGB']
speeds=[5,5,5,5,5,5]
FoV=[100,100,100, 80,80,100]
for i in range(DroneNum):
    Drones.append(Drone(id=i,sensor=sensors[i], FoV=FoV[i],speed=speeds[i]))
Drones=Auction(Area, Fire, Drones,EFA, Res,Tasks)
print(f"see Drones cost {[Drones[i].coverarea for i in range(len(Drones))]}")
TrackDraw(Drones, EFA, Area, Fire)
#EFAdict=defaultdict(list)
## p, xn, yn=Voronoi(EFA)
# imshow(EFA)
# plt.show()
# print(f"FQ scale {scale}")
# for i in [init,init+9]:
#     time=i*timestamp
#     file=f"{dir}/{foldername}/output/{foldername}_{time}_Perimeters.shp"
#     data=gpd.read_file(file)
#     area_map_,prim= get_raster(data, scale, BunsiteBound, NF=-1, F=1, P=2, Res=30)
#     EFA,x,y=UpadteEFA(Pmap, area_map_,time,EFA)
#     EFAdict[time]=[x,y]
#     Primdict[time]=prim
#     #print(f"Time is {time} {[x,y]}")
#     Pmap=area_map_



































