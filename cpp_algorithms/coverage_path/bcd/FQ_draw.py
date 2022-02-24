
import numpy as np
import matplotlib.pyplot as plt
import sys
import geopandas as gpd
sys.path.append('../../../')
from cpp_algorithms.common_helpers import imshow, imshow_scatter
from shapely.geometry import Point, Polygon
from pathlib import Path
# from zipfile import ZipFile
from geopy import distance
from skimage import measure
from skimage.draw import polygon
import shapely
import shapely.geometry
from collections import defaultdict
shape=[]

from skimage.draw import polygon
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
    Res=1
    mn=np.array(BunsiteBound[:2])
    sh=(np.array(BunsiteBound[2:])-mn)/Res# FQ change the boundary
    sh=np.int64(sh)
    p = np.full(sh,-1) #FQ change that
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
        r,c = polygon(*mix.T,sh)   #fq? should reverse that? 
        p[r,c] = 0  
    return p

def Firefront(maps):
    cu_map=maps[0]
    fu_map=maps[1]
    return cu_map-fu_map

def DrawStrip(startpoint):
    pass
def draw(area_map_):
    img=area_map_
    matrix = np.full(img.shape, 0, dtype=np.uint8)
    matrix[img == -1] = 255
    imshow(matrix, figsize=(8, 8), cmap="cividis")   
    plt.show(block=True)

def UpadteEFA(Pmap, area_map,time,EFA):
    if len(Pmap)==1:
        img=area_map
        EFA = np.full(img.shape, 0,dtype=np.uint8)
        EFA[img == 0] = time
        x,y= np.where(img==0)
    else:
        tmp=area_map-Pmap
        TMP = np.full(tmp.shape, 0, dtype=np.uint8)
        TMP[tmp==1] = time
        x,y=np.where(TMP==time)
        EFA=EFA+TMP
    return EFA, x,y
def logEFA(EFAdict, filename, shape, Res):
    f = open(filename, "w") 
    for key in EFAdict.keys():
        row=np.array(EFAdict[key])
        row=row
        f.write(f"{key} {list(row[0])} {list(row[1])}\n")
        #print(f"{key} {list(row[0])} {list(row[1])}")
    f.close() 
def GetEFA(filename):
    lines = []
    with open(filename) as f:
        lines = f.readlines()  
    print(len(lines)) 
    for line in lines: 
        print(f"FQ {line}")


dir='/home/fangqiliu/eclipse-workspace/Fire_simulation/Fire_sim/farsite/Rxfire/Burn_1/output/'
map2=[]
Pmap=[0]
EFAdict=defaultdict(list)
EFA=[]
BunsiteBound=(701235.0,4308525.0,704355.0,4311675.0)
scale = get_scale(BunsiteBound, meter=1)
print(f"FQ scale {scale}")
for i in range(1,int(50/3)):
    time=i*3
    file=f"{dir}Burn_1_{time}_Perimeters.shp"
    data=gpd.read_file(file)
    area_map_= get_raster(data, scale, BunsiteBound)
    EFA,x,y=UpadteEFA(Pmap, area_map_,time,EFA)
    EFAdict[time]=[x,y]
    Pmap=area_map_
shape=area_map_.shape
logEFA(EFAdict, "EFAdict.csv",shape, 10)
GetEFA("EFAdict.csv")


# EFA[EFA==0]=255
# print(EFAdict)
# imshow(EFA, figsize=(8, 8), cmap="cividis")   
# plt.show()
