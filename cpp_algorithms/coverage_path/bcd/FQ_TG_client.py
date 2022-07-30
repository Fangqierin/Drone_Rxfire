from random import choice
import threading 
import zmq 
import time
from pyknow import *
import geopandas as gpd
import os
from collections import defaultdict
import subprocess
from FQ_Task_Generator import  TaskManager 
from FQ_GenTask import GetEFA
from FQ_Firesim import DyFarsitefile, ClearBarrier, CreateDyRxfire
#from cpp_algorithms.common_helpers import imshow, imshow_scatter
from bcd_helper import imshow, imshow_scatter
import matplotlib.pyplot as plt
import re
import pandas as pd
import fiona
import json
import shapely
import numpy as np
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

def GetFirSim(bardata, BunsiteBound, foldername, MockName,dir, time=10, simdur=60,step=1, Res=10): # the location of the fire line, the length of the fireline and the number of the fires. 
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
            if  st+tt==time:
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
    DyFarsitefile(foldername, dir,time,simdur) # To run the simulation! 
    ########Do the next 40 minutes simulation! Then read the file, get EFA.
    EFA,EFAdict,Primdict = GetEFA(time,simdur,Bursite,dir,foldername, step=step,Res=Res)
    return EFA, EFAdict, Primdict

def DrawTask(tasks, EFAM):
    TaskMap=np.full(EFAM.shape, -1)
    codict={'BM':0,'FT':1,'FI':2, 'B':3}
    for key, mm in tasks.items():
        x,y=key
        if len(list(mm.keys()))>1:
            TaskMap[x,y]=codict['B']
        else:
            m=list(mm.keys())[0]
            TaskMap[x,y]=codict[m]
    imshow(TaskMap)
    plt.show()
    
        #print(f"x, y{x} {y}")
        
        
def DeclareGrid( TG, EFAM,tasks, Missions, init=1 ):
    rows,columns=EFAM.shape
    for x in range(rows):
        for y in range(columns):
            if EFAM[x,y]==1:
                st='B'
            else:
                st='N'
            TG.declare(Grid(id=(x,y),state=st, EFA=EFAM[x,y],time=0))
    #TG.declare(Phase2((1)))
    TG.run()
    print(tasks)
    DrawTask(tasks,EFAM) 

    
    
'''
See the task generation generated 'Taskdict' which is only used in Decomposition function, 
We can define a new taskdict, and update that in Decomposition method! 
##########################

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
    #Here is tricky, I sort it by its period, so it grid will not overlapped
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
                perTable[x[i],y[i]]=period # Get the maximum period!
                xc=int(x[i]/sc); yc=int(y[i]/sc)
                Area[(xc,yc)].append([x[i],y[i],mm[1][0]])
                p[x[i],y[i]]=cc#xc+yc+cc*2
        AreaPara[mm[0]]=Area
        cc=cc+1
    #cluster_imshow(p,sc=sc)
    #plt.show()
    return AreaPara
'''
    
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
    Res=10
    file='CARB_BurnUnits/CARB_BurnUnits.shp'
    data=gpd.read_file(f"{dir}/{file}")
    Bardata=ClearBarrier(data)
    #CreateDyRxfire(Bardata, Bursite,'FQ_burn',dir, [2])
    #print(f"see {Bardata}")
    # Before, we need to write the ignition file into the 
    EFA,EFAdict,Primdict =GetFirSim(Bardata, Bursite,  foldername, 'FQ_burn', dir, Res=Res,time=40)                  
    imshow(EFA)
    plt.show()
    #print(f"EFA: {EFA}")
    #print(f"EFAdict: {EFAdict}")
    TM=TaskManager(Missions)
    tasks=TM.DeclareGrid(EFA,init=1)
    DrawTask(tasks,EFA) 
    #print(f"Primdict: {Primdict}")



