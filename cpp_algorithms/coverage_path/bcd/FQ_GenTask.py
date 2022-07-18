
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
#from FQ_TaskAllocation import Decomposition, Auction, TrackDraw, Drone, Voronoi

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

def get_raster(gpdf_final, scale=2000, BunsiteBound=0, NF=-1, F=1, P=2, Res=1):
    """
    Returns rasterised version for the projection
    """
    #Res=30
    mn=np.array(BunsiteBound[:2])
    sh=(np.array(BunsiteBound[2:])-mn)# FQ change the boundary
    sh[0]=math.ceil(sh[0])
    sh[1]=math.ceil(sh[1])
    sh=np.int64(sh/Res)
    p = np.full(sh,NF) #FQ change that
    empty=np.full(sh,NF) #for record perimeter
    #prim=[[]for i in range(2)]
    for i in range(len(gpdf_final)):
        shp=gpdf_final.loc[i, 'geometry']
        ext = np.array(shp.exterior.coords)
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
    # imshow(empty)
    # plt.show()
    # imshow(p)
    # plt.show()
    #print(f"FQ change to  {len(x)}")
    return p,[x,y]

def get_raster_task(gpdf_final, scale=2000, BunsiteBound=0, NF=0, F=1, P=1, Res=1):
    """
    Returns rasterised version for the projection
    """
    #Res=30
    mn=np.array(BunsiteBound[:2])
    sh=(np.array(BunsiteBound[2:])-mn)# FQ change the boundary
    sh[0]=math.ceil(sh[0])
    sh[1]=math.ceil(sh[1])
    sh=np.int64(sh/Res)
    p = np.full(sh,NF) #FQ change that
    empty=np.full(sh,NF) #for record perimeter
    #prim=[[]for i in range(2)]
    for i in range(len(gpdf_final)):
        shp=gpdf_final.loc[i, 'geometry']
        ext = np.array(shp.exterior.coords)
        mix = (ext - mn)
        mx=(np.array(BunsiteBound[2:])-mn).max()  #FQ changed that
        mix1=np.ceil(mix/Res)#*3
        mix1 = np.int64(mix1)
        r,c = polygon(*mix1.T,sh)   #fq? should reverse that? 
        p[r,c] = F
        r,c=mix1.T # for the fire perimeter 
        empty[r,c]=P
        mix2=np.floor(mix/Res)#*30
        mix2=np.int64(mix2)
        r,c = polygon(*mix2.T,sh)
        p[r,c] = F
        r,c = mix2.T
        empty[r, c]=P
    x,y=np.where(empty==P)
    # imshow(empty)
    # imshow(p)
    # plt.show()
    #print(f"FQ change to  {len(x)}")
    return p,empty
def Firefront(maps):
    cu_map=maps[0]
    fu_map=maps[1]
    return cu_map-fu_map

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
    f.close() 
def GetEFA(filename):
    lines = []
    with open(filename) as f:
        lines = f.readlines()  

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
        #print(f"see time {time}")
        area_map_,prim= get_raster(data, scale, BunsiteBound, NF=-1, F=1, P=2, Res=Res)
        EFA,x,y=UpadteEFA(Pmap, area_map_,time,EFA)
        EFAdict[time]=[x,y]
        Primdict[time]=prim
        Pmap=area_map_ 
        time=time+timestamp
    #shape=area_map_.shape
    #logEFA(EFAdict, Primdict,"EFAdict.csv",shape, 10)
    return EFA, EFAdict, Primdict

def GenTasks(init,end, BunsiteBound, dir, foldername, Missions, Res=10):#This is a temporal one! 
    scale = get_scale(BunsiteBound, meter=1)
    Taskdict={}
    timestamp=3
    time=init
    Pmap=[0]
    data=[]
    for time in [init, end ]:
        file=f"{dir}/{foldername}/output/{foldername}_{time}_Perimeters.shp"
        data.append(gpd.read_file(file))
    area_map1,prim1= get_raster_task(data[0], scale, BunsiteBound, NF=0, F=1, P=1, Res=Res)
    area_map2,prim2= get_raster_task(data[1], scale, BunsiteBound, NF=0, F=1, P=1, Res=Res)
    Taskdict['FI']=[init, area_map1,Missions['FI'][0],Missions['FI'][1]]
    #Taskdict['FL']=[init, prim1,Missions['FL'][0],Missions['FL'][1]]
    FTA=area_map2-area_map1+prim2
    FTA[FTA>0]=1
    #plt.show()
    Taskdict['FT']=[init, FTA,Missions['FT'][0],Missions['FT'][1]]
    BMA=np.full(area_map2.shape,0)
    BMA[area_map2==0]=1
    BMA[prim2==1]=0
    Taskdict['BM']=[init, BMA, Missions['BM'][0],Missions['BM'][1]]
    return Taskdict, area_map2.shape

class Sensor:
    def _init_(self, Type='ALL', Pixel=1080, AFoV=math.pi*(60/180)):
        pass
if __name__ == "__main__":
    Missions=defaultdict(dict)  #  Missions: period, priority, 
    Missions['BM']=[ 10, 1]  #Other area 
    Missions['FI']=[ 5, 1]  #track the fire 
    Missions['FT']=[ 3, 3]  # track the fire perimeter and the risky area (arrival within 10 min)
    Missions['FL']=[ 3, 2]  # FL is tracking the fire perimeter.
    Missions['FD']=[ 2, 3]
    ################ Get tasks.
    dir='/home/fangqiliu/eclipse-workspace_Python/Drone_path/CoveragePathPlanning-master/farsite/'
    foldername='FQ_burn'
    BunsiteBound=(701235.0,4308525.0,704355.0,4311675.0)
    BunsiteBound=(702374.0,4309425.0,703700.0,4310900.0)
    Bursite=(702460.0,4309700.0,703540.0,4310820.0 )
    
    
    
    
    # igfile='/home/fangqiliu/eclipse-workspace_Python/Drone_path/CoveragePathPlanning-master/farsite/Rxfire/Burn_1/input/FQburn'
    # PutIginition(igfile,dir)
    init=120
    end=150
    Res=10
    #EFA,EFAdict,Primdict =GetEFFNext(init,end,BunsiteBound,dir,foldername, Res=Res)
    Task_mission=GenTasks(init,end,Bursite,dir,foldername, Missions ,Res=Res)
    #print(Task_mission)
























