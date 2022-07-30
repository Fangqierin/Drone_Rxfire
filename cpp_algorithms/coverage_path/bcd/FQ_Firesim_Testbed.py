
import numpy as np
import matplotlib.pyplot as plt
import sys
import geopandas as gpd
sys.path.append('../../../')
#from cpp_algorithms.common_helpers import imshow, imshow_scatter
from bcd_helper import imshow, imshow_scatter
from shapely.geometry import Point, Polygon
from pathlib import Path
from pyproj import Transformer
# from zipfile import ZipFile
#from geopy import distance
#from skimage import measure
from skimage.draw import polygon
from shapely.geometry import Point, LineString, Polygon, MultiLineString,MultiPolygon
from shapely.ops import cascaded_union
import shapely
import shapely.geometry
from collections import defaultdict
from skimage.draw import polygon
import random 
import os
import re
import math
from time import sleep
from subprocess import PIPE, Popen
import subprocess
import json
import fiona
from shapely.geometry import shape 
import pandas as pd
import time
from FQ_GenTask import GetEFA
from FQ_Task_Generator import TG, DetectFire, tasks, Timer,Grid, Phase2
from FQ_TG_client import DeclareGrid

####################################### Rxfire
def CreateRectangle(x_dl,y_dl, weight,height):# gag 10 m
    x_r=x_dl+weight
    y_u=y_dl+height
    coords=[(x_dl,y_dl), (x_r,y_dl), (x_r,y_u), (x_dl,y_u), (x_dl, y_dl)]
    poly=Polygon(coords)
    return poly 

def ClearBarrier(data):      
    polygons=[]
    for index, row in data.iterrows():
        poly = row['geometry']
        #print(f"seeeee {poly}")
        polygons.append(poly)
    #print(f"whyyyyy {polygons[0]}")
    u=cascaded_union([polygons[0],polygons[1],polygons[2]])
    u2=cascaded_union([polygons[3],polygons[6]])
    x,y=u2.exterior.coords.xy
    newdata = gpd.GeoDataFrame()
    newdata['geometry'] = None
    newdata.loc[0, 'geometry'] = u2.boundary[0]
    newdata.loc[1, 'geometry'] = polygons[4].boundary
    newdata.loc[2, 'geometry'] = u.boundary
    #da2=gpd.read_file(f"{dir}/{foldername}/input/seelin.shp")
    return newdata
    # da2.plot()
    # plt.show()    
def WriteInput(foldername, dir,step=1,initime=(0,0),dura=(2,0), dis_res=5, pre_res=5): # The simulation time 100 =1:00) duration<24 hours!
    fin = open(f"{dir}/{foldername}/input/Burn.input", "w")
    with open(f"{dir}/Template_burn/input/Burn.input", "r") as ftmp:
        filedata = ftmp.readlines()
    checkwind=False
    FARSITE_START_TIME=''
    for line in filedata:
        if re.match(r'FARSITE_START_TIME:*',line):
            time=line[:-1].split(' ')[-1]
            checkwind=True
            if initime[0]<10:
                time=f"0{initime[0]}"
            else:
                time=f"{initime[0]}"
            if initime[1]<10:
                time=time+f"0{initime[1]}"
            else:
                time=time+f"{initime[1]}"
            FARSITE_START_TIME=r'05 04 '+f"{time}"
            fin.write(f"FARSITE_START_TIME: {FARSITE_START_TIME}\n")
        elif checkwind  and re.match(r'2013 5 4 '+f"{time}",line):
            wind=int(line[:-1].split(' ')[-3])
            winddric=int(line[:-1].split(' ')[-2])
            fin.write(line)
        elif re.match(r'FARSITE_END_TIME:*',line):
            ent=[]
            ent.append(initime[0]+dura[0])
            if initime[1]+dura[1]>60:
                ent[0]=ent[0]+1
                ent.append(initime[1]+dura[1]-60)
            else:
                ent.append(initime[1]+dura[1])
            if ent[0]<10:
                time=f"0{ent[0]}"
            else:
                time=f"{ent[0]}"
            if ent[1]<10:
                time=time+f"0{ent[1]}"
            else:
                time=time+f"{ent[1]}"
            fin.write(f"FARSITE_END_TIME: 05 04 {time}\n")
        elif re.match(r'FARSITE_TIMESTEP:*',line):
            fin.write(f"FARSITE_TIMESTEP: {step}\n")
        elif re.match(f"FARSITE_DISTANCE_RES: ",line):
            fin.write(f"FARSITE_DISTANCE_RES: {float(dis_res)}\n")
        elif re.match(f"FARSITE_PERIMETER_RES: ",line):
            fin.write(f"FARSITE_PERIMETER_RES: {float(pre_res)}\n")
        else:
            fin.write(line)
    fin.close()


def PutSite(data, UID, Len, Width,offset=20):
    if len(UID)==1:
        # Get Boundary: 
        # poly=data.loc[UID[0],'geometry']
        # poly=Polygon(poly)
        site=data.loc[UID[0],'geometry']
        x,y=site.coords.xy
        mx,my,bx,by=(min(x),min(y),max(x),max(y))
        #print(f"length: {max(x)-min(x)} width: {max(y)-min(y)}")
        #offset=20
        boundary=(int(min(x)+offset), int(min(y)+offset), int(min(x)+offset+Len), int(min(y)+offset+Width))
        mx,my,bx,by=boundary
        #poly=CreateRectangle(mx,my,Len,Width)
        coords=[(mx,my), (bx,my), (bx,by), (mx,by), (mx, my)]
        poly=LineString(coords)
        bound_space=(mx, my,bx,by)
        # if poly.geom_type=='Polygon':
        #     feature=[0]
        # else:[data.loc[i,'geometry'] for i in range(len(data))
        #     feature =[i for i in range(len(poly.geoms))] #
        bargdr = gpd.GeoDataFrame()
        bargdr['geometry'] = None
        bargdr.loc[0, 'geometry']=poly #, crs='EPSG:4326)
        # bargdr.plot()
        # plt.show()
        print(f"bargdr {bargdr}")
    #print(f"data", data, [data.loc[i,'geometry'] for i in range(len(data))])
    #bargdr = MultiPolygon([data[i] for i in range(len(data)) if i!=UID[0]])
    return bound_space, bargdr

def UpdateFireIgnition(foldername, dir,BunsiteBound,Grid_map, Res, Fire=1):   
    mx,my,_,_=BunsiteBound
    x,y=np.where(Grid_map==1)
    poly = gpd.GeoDataFrame()
    fires=[]
    for i in range(len(x)):
        llx,lly=mx+x[i]*Res,my+y[i]*Res
        fire=CreateRectangle(llx,lly,Res,Res)
        poly.loc[i,'geometry']=fire
    #poly=shapely.ops.unary_union(fires)
    #print(poly)
    gdr=poly
    # if poly.geom_type=='Polygon':
    #     feature=[0]
    # else:
    #     feature =[i for i in range(len(poly))] #
    # gdr = gpd.GeoDataFrame({'feature': feature, 'geometry': poly}) #, crs='EPSG:4326)
    # for i in range(len(gdr)):
    #     x,y=gdr.loc[i,'geometry'].exterior.coords.xy
    #     print(x,y)
    #     plt.plot(x,y)
    #     plt.scatter(x,y)
    #     plt.show()
    #print(f"see gdr {gdr}")
    # gdr.plot()
    # plt.show()
    
    gdr.to_file(f"{dir}/{foldername}/input/{foldername}.shp")    #------> This is the objective!!!!!! 

def StartTestBed(foldername, bargdr, dir):
    os.system(f"mkdir {dir}/{foldername}")
    os.system(f"cp -r {dir}/Template_burn/input {dir}/{foldername}")
    bargdr.to_file(f"{dir}/{foldername}/input/TestBed.shp")    #------> This is the objective!!!!!! 
    try:
        os.system(f"mkdir {dir}/{foldername}/output")
    except:
        os.system(f"rm  {dir}/{foldername}/output")
        os.system(f"mkdir {dir}/{foldername}/output")

def RunFarsite_Test(foldername, dir,time,simdur,step=1):   
    #write testfile
    tmp=[0,0]
    simdur=simdur+20  #I do not why it always stop earlier! 
    if simdur>=60:
        tmp[0]=simdur//60
        tmp[1]=simdur%60
    else:
        tmp[1]=simdur
    #print(f"Sim time {tmp}")
    WriteInput(foldername,dir,dura=tmp, step=step)
    f = open(f"{dir}/{foldername}/{foldername}_TEST.txt", "w")
    f.write(f"{dir}/{foldername}/input/Burn.lcp ")# write landscape
    f.write(f"{dir}/{foldername}/input/Burn.input ") # write input file
    f.write(f"{dir}/{foldername}/input/{foldername}.shp ")# write ignition fire
    f.write(f"{dir}/{foldername}/input/seelin.shp ")
    #f.write(f"{dir}/{foldername}/input/TestBed.shp ")   # We can change the barrier!!! 
    f.write(f"{dir}/{foldername}/output/{time} 0")   # Output
    f.close()
    print(f"Command: {dir}/src/TestFARSITE {dir}/{foldername}/{foldername}_TEST.txt")
    try:
        out=os.system(f"{dir}/src/TestFARSITE {dir}/{foldername}/{foldername}_TEST.txt")# >/dev/null 2>&1")
        #print(f"see out", out.read())
        print(f"Generate the fire simulation successfully! Simulation from {time} duration  {simdur} ")
    except:
        print(f"Got error when simulating the fire spread")
    
if __name__ == "__main__":
    dir='/home/fangqiliu/eclipse-workspace_Python/Drone_path/CoveragePathPlanning-master/farsite/Rxfire/Burn_1/output/Burn_1'
    igfile='/home/fangqiliu/eclipse-workspace_Python/Drone_path/CoveragePathPlanning-master/farsite/Rxfire/Burn_1/input/FQburn'
    dir='/home/fangqiliu/eclipse-workspace_Python/Drone_path/CoveragePathPlanning-master/farsite'
    file='CARB_BurnUnits/CARB_BurnUnits.shp'
    data=gpd.read_file(f"{dir}/{file}")
    data=ClearBarrier(data)
    Missions=defaultdict(dict)  #  Missions: period, priority, 
    Missions['BM']=[ 10, 1]  #Other area 
    Missions['FI']=[ 5, 1]  #track the fire 
    Missions['FT']=[ 3, 3]  # track the fire perimeter and the risky area (arrival within 10 min)
    Missions['FL']=[ 3, 2]  # FL is tracking the fire perimeter.
    Missions['FD']=[ 5, 3]
    print(data)
    #space=10
    Res=5 # Grid size
    Len=8
    Width=6
    Grid_map=np.full((8,6),0)
    Grid_map[0,1]=1
    Grid_map[1,1]=1
    Grid_map[3,1]=1
    Grid_map[2,1]=1
    #Grid_map[2,3]=1
    # imshow(Grid_map)
    # plt.show()
    BunsiteBound,bargdr=PutSite(data, [2], Len*Res,Width*Res)
    StartTestBed('FQ_test', bargdr, dir)
    UpdateFireIgnition('FQ_test', dir, BunsiteBound,Grid_map, Res,Fire=1)
    simdur=40
    RunFarsite_Test('FQ_test', dir,0,simdur=simdur)
    #BunsiteBound=(702460.0,4309700.0,702860.0,4310200.0 )
    #data=CreateWildfie(BunsiteBound,1)
    #WriteInput('FQ_burn', dir)  # Just read somthing. 
    #CreateRxfire(data, BunsiteBound,'FQ_burn',dir, [2]) # Create the ignition file 
    #Farsitefile('FQ_burn',dir)
    EFA,EFAdict,Primdict = GetEFA(0,simdur,BunsiteBound,dir,'FQ_test',Res=Res)
    #imshow(EFA)
    #plt.show()
    DeclareGrid(TG, EFA,init=1, tasks=tasks,Missions=Missions)

########################################### Old code, for something else. 