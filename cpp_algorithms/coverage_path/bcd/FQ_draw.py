
import numpy as np
import matplotlib.pyplot as plt
import sys
import geopandas as gpd
sys.path.append('../../../')
from cpp_algorithms.common_helpers import imshow, imshow_scatter
from shapely.geometry import Point, Polygon
from pathlib import Path
# from zipfile import ZipFile
#from geopy import distance
#from skimage import measure
from skimage.draw import polygon
import shapely
import shapely.geometry
from collections import defaultdict
from skimage.draw import polygon
import os
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

def get_raster(gpdf_final, scale=2000, BunsiteBound=0,CRS = f"EPSG:4326"):
    """
    Returns rasterised version for the projection
    """
    Res=10
    mn=np.array(BunsiteBound[:2])
    sh=(np.array(BunsiteBound[2:])-mn)/Res# FQ change the boundary
    sh=np.int64(sh)
    p = np.full(sh,-1) #FQ change that
    empty=np.full(sh,-1) #for record perimeter
    prim=[[]for i in range(2)]
    for i in range(len(gpdf_final)):
        tmp=gpdf_final
        try:
            shp = tmp.to_crs(crs=CRS)
        except:
            shp = tmp.set_crs(crs="EPSG:4326")
        ext = np.array(shp.geometry[i].exterior.coords).copy()
        mix = (ext - mn)/Res
        mx = mix.max()
        mx=(np.array(BunsiteBound[2:])-mn).max()  #FQ changed that
        mix *= scale/mx
        mix = np.int64(mix)
        br,bc=mix.T # for the fire perimeter 
        #print(f"FQ see len prim {len(br)}")
        empty[br, bc]=0
        #print(f"FQ see {mix} {prim[0]}, {[0]+bc} {bc.shape}")
        prim[0]=prim[0]+list(br)
        prim[1]=prim[1]+list(bc)
        r,c = polygon(*mix.T,sh)   #fq? should reverse that? 
        p[r,c] = 0
    x,y=np.where(empty==0)
    imshow(empty)
    plt.show()
    print(f"FQ change to  {len(x)}")
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

def UpadteEFA(Pmap, area_map,time,EFA):
    if len(Pmap)==1:
        img=area_map
        EFA = np.full(img.shape, -1,dtype=np.uint8)
        EFA[img == 0] = time
        x,y= np.where(img==0)
    else:
        tmp=area_map-Pmap
        EFA[tmp==1]=time
        x,y=np.where(EFA==time)
        
    return EFA, x,y
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

shape=[] 
dir='/home/fangqiliu/eclipse-workspace_Python/Drone_path/CoveragePathPlanning-master/farsite/Rxfire/Burn_1/output/Burn_1'
map2=[]
Pmap=[0]
EFAdict=defaultdict(list)
Primdict=defaultdict(list)
EFA=[]
BunsiteBound=(701235.0,4308525.0,704355.0,4311675.0)
scale = get_scale(BunsiteBound, meter=1)
igfile='/home/fangqiliu/eclipse-workspace_Python/Drone_path/CoveragePathPlanning-master/farsite/Rxfire/Burn_1/input/FQburn'
PutIginition(igfile,dir)
timestamp=3
print(f"FQ scale {scale}")
for i in range(0,int(63/3)):
    time=i*timestamp
    file=f"{dir}_{time}_Perimeters.shp"
    data=gpd.read_file(file)
    area_map_,prim= get_raster(data, scale, BunsiteBound)
    EFA,x,y=UpadteEFA(Pmap, area_map_,time,EFA)
    EFAdict[time]=[x,y]
    Primdict[time]=prim
    #print(f"Time is {time} {[x,y]}")
    Pmap=area_map_
shape=area_map_.shape
logEFA(EFAdict, Primdict,"EFAdict.csv",shape, 10)
GetEFA("EFAdict.csv")

#EFA[EFA==-1]=255
print(EFAdict)
imshow(EFA, figsize=(8, 8), cmap="cividis")   
plt.show()
