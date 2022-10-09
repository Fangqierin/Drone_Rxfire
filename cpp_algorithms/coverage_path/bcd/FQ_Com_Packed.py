import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from bcd_helper import imshow, imshow_scatter
import math
from collections import defaultdict
import random
import time
import geopandas as gpd
from FQ_TaskAllocation_WPC import  Auction_WPC, TrackDraw
from FQ_Drones_Info import Sensor, Drone, ReadSen_PPM,LoadDrones
from FQ_Task_Generator import  TaskManager 
from FQ_Firesim import DyFarsitefile, ClearBarrier, CreateDyRxfire
from FQ_TG_client import GetFirSim, Decomposition
#from FQ_TaskAllocation_Comp import Auction_Comp
from FQ_FlightPlanning_Comp import AllComp
import sys
#from FQ_FlightPlanning_boundary import FlightPlanner

# we need know its flying height, sensor coverage! cover reward!!!!!!!!!!!
    
if __name__ == "__main__":
    try:
        wind=int(sys.argv[1]);
        STtime=str(sys.argv[2]);
    except: 
        wind=15
        STtime=20
        #Plantime=60*20

    Missions=defaultdict(dict)  #  Missions: id, period, priority, 
    Missions['BM']=[ 10, 2]  #Other area 
    Missions['FI']=[ 5, 1]  #track the fire 
    Missions['FT']=[ 2.5, 3]  # track the fire perimeter and the risky area (arrival within 10 min)
    ################ Get tasks.
    #dir='/home/fangqiliu/eclipse-workspace_Python/Drone_path/CoveragePathPlanning-master/farsite'
    dir='../../../farsite'
    logfile=f"logtask.txt"
    IFLOG=True
    if IFLOG:
        logfile=open(logfile, "w")
    DecomposeSize=50
    #Bursite=(702460.0,4309700.0,703540.0,4310820.0 )
    Bursite=(702460.0,4309700.0,702860.0,4310200.0 )
    Res=10
    ########################## Do decomposition~ 
    file='CARB_BurnUnits/CARB_BurnUnits.shp'
    data=gpd.read_file(f"{dir}/{file}")
    Bardata=ClearBarrier(data)
    #CreateDyRxfire(Bardata,fir_name,dir, [2],wind=wind)
    fir_name=f"FQ_Sim_{wind}"
    foldername=f"FQ_Tmp_{wind}_{STtime}"
    EFA,EFAdict,bound =GetFirSim(Bardata,  foldername, fir_name, dir, Bursite, Res=Res,time=STtime,wind=wind)                  
    TM=TaskManager(Missions)
    tasks=TM.DeclareGridEFA(EFA,init=0)
    ########################################
    sensorfile='Data/sensor_info.csv'
    PPMfile='Data/PPM_table.csv'
    DroneNum=6
    speeds=[5,5,5,5,3,5]
    loiter=[1,1,1,1,1,1]
    ranges=[300,300,500,300,300,500]
    GCloc=(0,500)
    GCloc=(0,0)
    inloc=(0,0,0)
    sensgrp=[['ZENMUSE_XT2_t','ZENMUSE_XT2_r'],['DJI_Air2S'],['ZENMUSE_XT2_t','ZENMUSE_XT2_r'],['DJI_Air2S'],['ZENMUSE_XT2_t','ZENMUSE_XT2_r'],['ZENMUSE_XT2_t','ZENMUSE_XT2_r']]
    Drones=LoadDrones(sensorfile,PPMfile,DroneNum, speeds, sensgrp, Res,loiter,ranges)
    init=0;Plantime=60*20
    #Do decomposition~  1: Normal 2: Inter 3: Area 4: Voronoi
    ## GWP= 1: WPC_SetCover; 2: WPC_Adjust; 3: Regular
    ###PLAN 0, Ours 1: DD+Return; 2: Reward_Driven+Return; 3: DL+DD+Return; 4: DL+RD+Return; ## 5: DL+CO+Return; 6: DL+CO+NoReturn 
    logfile=f"./Results/log9_{wind}_{STtime}"
    log=open(logfile, "w")
    Simfile=f"./Results/Simple9_{wind}_{STtime}"
    Simlog=open(Simfile,"w")
    
    # TANum=1;GWP=1;FPnum=0
    # Rewardl, Pl, Runtiml=AllComp( TANum,GWP,FPnum,Drones,init, Plantime,inloc,GCloc, Missions,DecomposeSize,EFA, Res,tasks,log)
    #Simlog.write(f"Sum {TANum} {GWP} {FPnum} {sum(Rewardl)} {sum(Pl)} {max(Runtiml)} {mean(Runtiml)}\n")
    #########################
    for TANum in [1,2,3,4]:
        for GWP in [1,2,3]:
            for FPnum in [0,1,2,3,4,5,6]:
                Rewardl, Pl, Runtiml,LogTask,LogMiss,LogReward=AllComp( TANum,GWP,FPnum,Drones,init, Plantime,inloc,GCloc, Missions,DecomposeSize,EFA, Res,tasks,log)
                Simlog.write(f"Sum {TANum} {GWP} {FPnum} {sum(Rewardl)} {sum(Pl)} {max(Runtiml)} {np.mean(Runtiml)}\n")
                for i in range(len(Rewardl)):
                    Simlog.write(f"Drone {TANum} {GWP} {FPnum} {i}; {Rewardl[i]} {Pl[i]} {Runtiml[i]}\n")
                    #Simlog.write(f"Tasks {i}\n")
                    for tak, timeNum in LogTask[i].items():
                        Simlog.write(f"Tasks {TANum} {GWP} {FPnum} {i}; {tak}; {sum(list(timeNum.values()))}; ")
                        for time, num in timeNum.items():
                            Simlog.write(f"{int(time)} {num}; ")
                        Simlog.write(f"\n")
                    #Simlog.write(f"Miss {i} {TANum} {GWP} {FPnum}\n")
                    for tak, timeNum in LogMiss[i].items():
                        Simlog.write(f"Miss {TANum} {GWP} {FPnum} {i}; {TANum} {GWP} {FPnum} {tak}; {sum(list(timeNum.values()))}; ")
                        for time, num in timeNum.items():
                            Simlog.write(f"{int(time)} {num}; ")
                        Simlog.write(f"\n")
                    Simlog.write(f"Reward {TANum} {GWP} {FPnum} {i}; ")
                    for time, Rdset in LogReward[i].items():
                        r,s,p=Rdset
                        Simlog.write(f"{int(time)} {r} {s} {p}; ")
                    Simlog.write(f"\n")
     #################################
    log.close()
    Simlog.close()






