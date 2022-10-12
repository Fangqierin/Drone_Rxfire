from random import choice
import threading 
import zmq 
import time
#from pyknow import *
import geopandas as gpd
import os
from collections import defaultdict
import subprocess
from FQ_Task_Generator import  TaskManager 
from FQ_GenTask import GetEFA
from FQ_TaskAllocation_WPC import Auction_WPC, TrackDraw
#from FQ_Task_Generator import  TaskManager 
from FQ_Firesim import DyFarsitefile, ClearBarrier, CreateDyRxfire
#from cpp_algorithms.common_helpers import imshow, imshow_scatter
from bcd_helper import imshow, imshow_scatter,imshow_Task
import matplotlib.pyplot as plt
import re
import pandas as pd
import fiona
import json
import shapely
import numpy as np
from FQ_Drones_Info import Sensor, Drone, ReadSen_PPM, LoadDrones

#TG=TaskGenerator()
#p=subprocess.Popen(f"pwd", )
# p=subprocess.Popen(f"python3 ./FQ_Task_Generator.py", shell=True)
#p.wait()
# print(f"TASKS: {tasks}")
# TG.declare(DetectFire(id=1,yes=True, time=1))
# TG.declare(DetectFire(id=2,yes=False,time=3))
# print("===2 run===")
# TG.run()
# print(TG.facts)
# print(f"TASKS: {tasks}")
# TG.declare(DetectFire(id=0,yes=True,time=4))
# print("===3 run===")
# TG.run()
# #print(TG.facts)
# print(f"TASKS: {tasks}")
# TG.declare(DetectFire(id=1,yes=False,time=5))
# TG.declare(DetectFire(id=2,yes=True,time=5))
# print("===4 run===")
# TG.run()
# print(TG.facts)
# print(f"TASKS: {tasks}")
# TG.declare(DetectFire(id=2,yes=True,time=6))
# TG.declare(DetectFire(id=2,yes=False,time=8))
# print("===5 run===")
# TG.run()
# print(TG.facts)
# print(f"TASKS: {tasks}")
# print(TG.facts)

def GetFirSim(bardata, foldername, MockName,dir, Bursite, time=10, simdur=60,step=1, Res=10,wind=5,direction=270,sed=0,Inputdict=[]): # the location of the fire line, the length of the fireline and the number of the fires. 
    os.system(f"mkdir {dir}/{foldername}")
    os.system(f"cp -r {dir}/Template_burn/input {dir}/{foldername}")
    bardata.to_file(f"{dir}/{foldername}/input/seelin.shp")
    #### Before, we need write fire ignition file 
    f = []
    for (dirpath, dirnames, filenames) in os.walk(f"{dir}/{MockName}/output/"):
        f.extend(filenames)
        break
    f.sort()
    for file in f:
        if re.search('_Perimeters',file) and len(file.split('_'))>2:
            st=int(file.split('_')[0])
            tt=int(file.split('_')[1])
            if  tt==time:
                collection = list(fiona.open(f"{dir}/{MockName}/output/{file}",'r'))
                df1 = pd.DataFrame(collection)
                def isvalid(geom):
                    if len(geom['coordinates'][0])>2:
                        return 1
                    else:
                        return 0
                df1['isvalid'] = df1['geometry'].apply(lambda x: isvalid(x))
                df1 = df1[df1['isvalid'] == 1]
                collection = json.loads(df1.to_json(orient='records'))
                #Convert to geodataframe
                data = gpd.GeoDataFrame.from_features(collection)
                geoms=[data.loc[i, 'geometry'] for i in range(len(data))]
                data2=shapely.ops.unary_union([geom if geom.is_valid else geom.buffer(0) for geom in geoms]) 
                if data2.geom_type=='MultiPolygon':
                    features = [i for i in range(len(data2))]
                    gdr = gpd.GeoDataFrame({'feature': features, 'geometry': data2}) #, crs='EPSG:4326)
                else:
                    gdr = gpd.GeoDataFrame({'feature': [0], 'geometry': data2}) #, crs='EPSG:4326)
                gdr.to_file(f"{dir}/{foldername}/input/{foldername}.shp")    #------> This is the objective!!!!!! 
                break
    try:
        os.system(f"mkdir {dir}/{foldername}/output")
    except:
        os.system(f"rm -r {dir}/{foldername}/output")
        os.system(f"mkdir {dir}/{foldername}/output")
    DyFarsitefile(foldername, dir,0,simdur,wind=wind,direction=direction,seed=sed,Inputdict=Inputdict) # To run the simulation! 
    ########Do the next 40 minutes simulation! Then read the file, get EFA.
    EFA,EFAdict,bound = GetEFA(0,simdur,Bursite,dir,foldername, step=step,Res=Res)
    #### Crop the EFA 
    mx,my,ax,ay=np.array(bound)//Res-np.array(Bursite)//Res
    #if mx<0: # Remove 
    if mx<0:
        EFA=np.delete(EFA,slice(-int(mx)),0) # [:mx] Delete first mx column
    if ax>0:
        EFA=np.delete(EFA,slice(-int(ax),None),0) # [-2:] Delete first mx column
    if my<0:
        EFA=np.delete(EFA,slice(-int(my)),1) # [:mx] Delete first mx column
    if ay>0:
        EFA=np.delete(EFA,slice(-int(ay),None),1) # [-2:] Delete first mx column
    return EFA, EFAdict, bound



    
'''
See the task generation generated 'Taskdict' which is only used in Decomposition function, 
We can define a new taskdict, and update that in Decomposition method! 
##########################
'''
def Decomposition(DecomposeSize, Res,Tasks):  # Return the dictionary of grids and area
    #print(f"get task {tasks}")
    DecomposeSize=int((DecomposeSize/Res)*Res)
    sc=DecomposeSize/Res
    AreaPara=defaultdict(list)
    cc=0
    for grid, mm in Tasks.items():
        for m in list(mm.keys()):
            x,y=grid
            xc=int(x//sc); yc=int(y//sc)
            if len(list(mm.keys()))==1: 
                AreaPara[(xc,yc),m].append((x,y,mm[m]))
            else:
                AreaPara[(xc,yc),m, '1'].append((x,y,mm[m]))
    return AreaPara

if __name__ == "__main__":
    Missions=defaultdict(dict)  #  Missions: period, priority, 
    Missions['BM']=[ 10, 1]  #Other area 
    Missions['FI']=[ 5, 1]  #track the fire 
    Missions['FT']=[ 3, 3]  # track the fire perimeter and the risky area (arrival within 10 min)
    Missions['FL']=[ 3, 2]  # FL is tracking the fire perimeter.
    Missions['FD']=[ 5, 3]
    ################ Get tasks.
    dir='/home/fangqiliu/eclipse-workspace_Python/Drone_path/CoveragePathPlanning-master/farsite'
    foldername='FQ_sim'
    #Bursite=(702460.0,4309700.0,703540.0,4310820.0 )
    Bursite=(702460.0,4309700.0,702860.0,4310200.0 )
    DecomposeSize=50
    Res=10
    file='CARB_BurnUnits/CARB_BurnUnits.shp'
    data=gpd.read_file(f"{dir}/{file}")
    Bardata=ClearBarrier(data)
    #CreateDyRxfire(Bardata,'FQ_burn',dir, [2])
    EFA,EFAdict,Primdict =GetFirSim(Bardata,  foldername, 'FQ_burn', dir, Bursite, Res=Res,time=40)                  
    # imshow(EFA)
    # plt.show()
    TM=TaskManager(Missions)
    tasks=TM.DeclareGridEFA(EFA,init=0)
    #DrawTask(tasks,EFA) 
    ########################################
    sensorfile='Data/sensor_info.csv'
    PPMfile='Data/PPM_table.csv'
    DroneNum=5
    speeds=[5,5,5,5,3,3]
    loiter=[1,2,1,1,1]
    ranges=[300,200,500,300,300]
    #ranges=[200,100,100,300,100]
    GCloc=(0,500)
    sensgrp=[['ZENMUSE_XT2_t','ZENMUSE_XT2_r'],['DJI_Air2S'],['ZENMUSE_XT2_t','ZENMUSE_XT2_r'],['DJI_Air2S'],['ZENMUSE_XT2_t','ZENMUSE_XT2_r']]
    Drones=LoadDrones(sensorfile,PPMfile,DroneNum, speeds, sensgrp, Res,loiter,ranges)
    ####################### Generate flight plan! 
    AreaPara=Decomposition(DecomposeSize,Res,tasks) # Need put tasks there!!!
    # for k in list(AreaPara.keys()):
    #     print(k)
    Drones,Bidders=Auction(AreaPara, Drones, Res,Missions,3, EFA,GCloc)
    print(f"see Drones cost {[Bidders[i].coverarea for i in range(len(Drones))]}")

    TrackDraw(Drones, EFA)
######################################
    ################ Here, 
    #I allow a grid have multiple tasks, next, I need to modify auction! 
    #If another tasks, check if it can be overload, if not, add more workload.

    #print(f"get {AreaPara}")
    # TaskMap=np.full(EFA.shape, -1)
    # cc=0
    # print(len(AreaPara))
    # for key, cor in AreaPara.items():
    #     x=[i[0] for i in cor]
    #     y=[i[1] for i in cor]
    #     print(f"see key {key} {x} {y}")
    #     TaskMap[x,y]=cc
    #     cc=cc+10
    # imshow(TaskMap)
    # plt.show()
    ####### We need the simulation the real observation! 
    #everytime uploading data, we can get the task update! 
    
    
    











