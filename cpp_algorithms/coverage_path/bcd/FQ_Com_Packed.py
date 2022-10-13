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
        STtime=int(sys.argv[2]);
        Unit=int(sys.argv[3])
        #print("unit",Unit)
    except: 
        wind=15
        STtime=1
        Unit=2
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
    Res=10
    ########################## Do decomposition~ 
    file='CARB_BurnUnits/CARB_BurnUnits.shp'
    data=gpd.read_file(f"{dir}/{file}")
    Bardata=ClearBarrier(data)
    #CreateDyRxfire(Bardata,fir_name,dir, [2],wind=wind)
    #CreateRandomRxfire(Bardata,fir_name,dir, [2],wind=wind,sed=seed,Inputdict=Inputdict)
    
    ########################################
    sensorfile='Data/sensor_info.csv'
    PPMfile='Data/PPM_table.csv'
    DroneNum=6
    speeds=[5,5,5,5,5,5]
    loiter=[1,1,1,1,1,1]
    ranges=[500,300,500,300,300,500]
    GCloc=(0,500)
    GCloc=(0,0)
    inloc=(0,0,0)
    sensgrp=[['ZENMUSE_XT2_t','ZENMUSE_XT2_r'],['DJI_Air2S'],['ZENMUSE_XT2_t','ZENMUSE_XT2_r'],['DJI_Air2S'],['ZENMUSE_XT2_t','ZENMUSE_XT2_r'],['ZENMUSE_XT2_t','ZENMUSE_XT2_r']]
    Drones=LoadDrones(sensorfile,PPMfile,DroneNum, speeds, sensgrp, Res,loiter,ranges)
    init=0;Plantime=60*20
    #Do decomposition~  1: Normal 2: Inter 3: Area 4: Voronoi
    ## GWP= 1: WPC_SetCover; 2: WPC_Adjust; 3: Regular
    ###PLAN 0, Ours 1: DD+Return; 2: Reward_Driven+Return; 3: DL+DD+Return; 4: DL+RD+Return; ## 5: DL+CO+Return; 6: DL+CO+NoReturn 
    logfile=f"./Results/log14_{Unit}_{wind}_{STtime}"
    log=open(logfile, "w")
    Simfile=f"./Results/Simple14_{Unit}_{wind}_{STtime}"
    Simlog=open(Simfile,"w")
    
    # TANum=1;GWP=1;FPnum=0
    # Rewardl, Pl, Runtiml=AllComp( TANum,GWP,FPnum,Drones,init, Plantime,inloc,GCloc, Missions,DecomposeSize,EFA, Res,tasks,log)
    #Simlog.write(f"Sum {TANum} {GWP} {FPnum} {sum(Rewardl)} {sum(Pl)} {max(Runtiml)} {mean(Runtiml)}\n")
    #########################
    for seed in range(10):
        ############################### For random seed
        #We have 8 time slots each of 20 minutes 
        slots=['0000','0020','0040','0100','0120','0140','0200','0220']
        erwind=3
        random.seed(seed)
        windset=np.array([random.randint(-erwind,erwind) for i in range(8)])+np.array([wind for i in range(8)])
        direction=270
        erdir=10
        dirset=np.array([random.randint(-erdir,erdir) for i in range(8)])+np.array([direction for i in range(8)])
        Winddict={}
        Inputdict={}
        ct=0
        for i in range(0,160,20):
            Winddict[i]=(windset[ct],dirset[ct])
            Inputdict[slots[ct]]=(windset[ct],dirset[ct])
            ct=ct+1
        ########################################################
        if Unit==1:
            fir_name=f"FQ_Rand_{wind}_{seed}"
            foldername=f"FQ_Tmp_{wind}_{STtime}_{seed}"
            Bursite=(702460.0,4309700.0,702860.0,4310200.0 )
        if Unit==2:
            fir_name=f"FQ_Rand_U2_{wind}_{seed}"
            foldername=f"FQ_Tmp_U2_{wind}_{STtime}_{seed}"
            Bursite=(702850.0,4310030.0,703250.0,4310360.0)
        if Unit==3:
            fir_name=f"FQ_Rand_U3_{wind}_{seed}"
            foldername=f"FQ_Tmp_U3_{wind}_{STtime}_{seed}"
            Bursite=(703000.0,4310230.0,703500.0,4310780.0)
        
        tt=min([k for k in list(Winddict.keys()) if k<=STtime])
        windd,directt=Winddict[tt]
        #print(f" {windd} {directt}")
        #CreateRandomRxfire(Bardata,fir_name,dir, [2],wind=wind,sed=seed,Inputdict=Inputdict)
        EFA,EFAdict,Primdict =GetFirSim(Bardata,  foldername, fir_name, dir, Bursite, Res=Res,time=STtime,wind=windd,direction=directt,sed=seed)                  
        #EFA,EFAdict,bound =GetFirSim(Bardata,  foldername, fir_name, dir, Bursite, Res=Res,time=STtime,wind=wind)                  
        TM=TaskManager(Missions)
        ct=time.time()
        tasks=TM.DeclareGridEFA(EFA,init=0)
        TaskGTime=time.time()-ct
        #TM.reset()
        Simlog.write(f"TaskGen {TaskGTime}\n")
        for TANum in  [1,2,3,4]:
            for GWP in [1,2,3]:
                for FPnum in [0,1,2,3,4,5,6]:
                    TaskATime,Rewardl, Pl, Runtiml,LogTask,LogMiss,LogReward=AllComp( TANum,GWP,FPnum,Drones,init, Plantime,inloc,GCloc, Missions,DecomposeSize,EFA, Res,tasks,log,seed)
                    Simlog.write(f"Sum {TANum} {GWP} {FPnum} {seed} {sum(Rewardl)} {sum(Pl)} {max(Runtiml)} {np.mean(Runtiml)} {TaskATime}\n")
                    print(f"Sum {TANum} {GWP} {FPnum} {seed} {sum(Rewardl)} {sum(Pl)} {max(Runtiml)} {np.mean(Runtiml)}\n")
                    for i in range(len(Rewardl)):
                        Simlog.write(f"Drone {TANum} {GWP} {FPnum} {seed} {i}; {Rewardl[i]} {Pl[i]} {Runtiml[i]}\n")
                        print(f"Drone {TANum} {GWP} {FPnum} {seed} {i}; {Rewardl[i]} {Pl[i]} {Runtiml[i]}\n")
                        #Simlog.write(f"Tasks {i}\n")
                        for tak, timeNum in LogTask[i].items():
                            Simlog.write(f"Tasks {TANum} {GWP} {FPnum} {seed} {i}; {tak}; {sum(list(timeNum.values()))}; ")
                            for tt, num in timeNum.items():
                                Simlog.write(f"{int(tt)} {num}; ")
                            Simlog.write(f"\n")
                        #Simlog.write(f"Miss {i} {TANum} {GWP} {FPnum}\n")
                        for tak, timeNum in LogMiss[i].items():
                            Simlog.write(f"Miss {TANum} {GWP} {FPnum} {seed} {i}; {TANum} {GWP} {FPnum} {tak}; {sum(list(timeNum.values()))}; ")
                            for tt, num in timeNum.items():
                                Simlog.write(f"{int(tt)} {num}; ")
                            Simlog.write(f"\n")
                        Simlog.write(f"Reward {TANum} {GWP} {FPnum} {seed} {i}; ")
                        for tt, Rdset in LogReward[i].items():
                            r,s,p=Rdset
                            Simlog.write(f"{int(tt)} {r} {s} {p}; ")
                        Simlog.write(f"\n")
     #################################
    log.close()
    Simlog.close()





