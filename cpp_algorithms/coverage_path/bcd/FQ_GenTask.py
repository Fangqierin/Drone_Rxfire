
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
    #imshow(empty)
    # imshow(p)
    # plt.show()
    #print(f"FQ change to  {len(x)}")
    return p,[x,y]
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

class Sensor:
    def _init_(self, Type='ALL', Pixel=1080, AFoV=math.pi*(60/180)):
        pass
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
    Bursite=(702474.4,4309711.3,703511.1,4310784.9 )
    # igfile='/home/fangqiliu/eclipse-workspace_Python/Drone_path/CoveragePathPlanning-master/farsite/Rxfire/Burn_1/input/FQburn'
    # PutIginition(igfile,dir)
    init=120
    end=150
    Res=10
    EFA,EFAdict,Primdict =GetEFFNext(init,end,BunsiteBound,dir,foldername, Res=Res)





































