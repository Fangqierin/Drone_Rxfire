import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from bcd_helper import imshow, imshow_EFA, DrawTask
import math
from collections import defaultdict
import random
import time
import geopandas as gpd
from FQ_TaskAllocation_WPC import  Auction_WPC, TrackDraw
from FQ_Drones_Info import Sensor, Drone, ReadSen_PPM,LoadDrones
from FQ_Task_Generator import  TaskManager 
from FQ_Firesim import DyFarsitefile, ClearBarrier, CreateDyRxfire,CreateRandomRxfire
from FQ_TG_client import GetFirSim, Decomposition
import copy
    
if __name__ == "__main__":
    Missions=defaultdict(dict)  #  Missions: id, period, priority, 
    Missions['BM']=[ 10, 2]  #Other area 
    Missions['FI']=[ 5, 1]  #track the fire 
    Missions['FT']=[ 3, 3]  # track the fire perimeter and the risky area (arrival within 10 min)
    #Missions['FL']=[ 3, 2]  # FL is tracking the fire perimeter.
    #Missions['FD']=[ 2, 3]
    ################ Get tasks.
    dir='/home/fangqiliu/eclipse-workspace_Python/Drone_path/CoveragePathPlanning-master/farsite'
    foldername='FQ_sim'
    #Bursite=(702460.0,4309700.0,703540.0,4310820.0 )
    #Bursite=(702500.0,4309700.0,702900.0,4310200.0 )
    Bursite3=(703000.0,4310230.0,703500.0,4310780.0)
    Bursite=(703000.0,4310230.0,703500.0,4310780.0)

    Res=10
    #########We should meet sometimes,################# Do decomposition~ 
    file='CARB_BurnUnits/CARB_BurnUnits.shp'
    data=gpd.read_file(f"{dir}/{file}")
    Bardata=ClearBarrier(data)
    wind=15; time=60; seed=1
    for wind in [20,25]:#[15]:
        for time in [60]:# [1,20, 40,60]:
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
                fir_name=f"FQ_Rand_U3_{wind}_{seed}"
                foldername=f"FQ_Tmp_U3_{wind}_{time}_{seed}"
                #CreateDyRxfire(Bardata,fir_name,dir, [2],wind=wind)
                CreateRandomRxfire(Bardata,fir_name,dir, [0],wind=wind,sed=seed,Inputdict=Inputdict)
                print(f"Finish create {wind} {seed}")  
                '''
                tt=min([k for k in list(Winddict.keys()) if k<=time])
                windd,directt=Winddict[tt]
                #print(f" {windd} {directt}")
                EFA,EFAdict,Primdict =GetFirSim(Bardata,  foldername, fir_name, dir, Bursite, Res=Res,time=time,wind=windd,direction=directt,sed=seed)                  
                imshow_EFA(EFA)
                #plt.show()
                #plt.savefig(f"EFA_{wind}_{time}.eps", bbox_inches='tight')
                TM=TaskManager(Missions)
                tasks=TM.DeclareGridEFA(EFA,init=0)
                #print(f" see tasks",TM.cur_tasks)
                #TM.reset()
                print(f"check {wind} time {time} seed {seed}")
                # DrawTask(tasks,EFA)
                # plt.show() 
                #
                #imshow_EFA(EFA)
                '''
                
            #plt.savefig(f"Task_{wind}_{time}.eps", bbox_inches='tight')

    #
    # def Cluster_Valid_Decision(self, DecomposeSize, Res,UpdateGrid):   # Check the missions in the bigger cluster!
    #     areas=[]
    #     sr=DecomposeSize/Res
    #     dlist=list(self.Mission_DL.items())  # Check the dl! 
    #     dlist.sort(key=lambda s: s[1]) # Get the earliest deadline. 
    #     if UpdateGrid:
    #         GetDL=None
    #         for m in dlist:
    #             if GetDL!=None:
    #                 if m[1]==GetDL and len(self.Mission_JR[m[0]][0])>0:
    #                     tmp=self.Mission_JR[m[0]][0]
    #                     for key, aa in self.Mission_SR[m[0]].items():
    #                         tmp=list(set(tmp)-set(aa)) # if it is stored? 
    #                     if len(tmp)>0:   
    #                          areas=list(set(areas+tmp))
    #             else:
    #                 if len(self.Mission_JR[m[0]][0])>0:   # check which mission has this deadline? 
    #                     areas=self.Mission_JR[m[0]][0] # Get the task areas 
    #                     for key, aa in self.Mission_SR[m[0]].items():
    #                         areas=list(set(areas)-set(aa)) # if it is stored? 
    #                     if len(areas)>0:    
    #                         GetDL=m[1]
    #                         mission=m
    #         ############ If Grid_tocov should maintain all tasks? If not, we do not need it every time!!! 
    #         CurGrid=set()
    #         for a in areas: 
    #             self.Grid_tocov[self.Area_grid[a]].append(a)
    #             CurGrid.add(self.Area_grid[a])
    #         #Select a grid to cover! 
    #         ###################################################
    #         CurGrid=list(CurGrid)
    #         print(f"see waypoint {self.wp} {CurGrid}")
    #         Diss=[self.Distance(self.wp, (g[0][0]*sr,g[0][1]*sr,0)) for g in CurGrid]
    #         adex=Diss.index(min(Diss)) 
    #         near_g=CurGrid[adex] 
    #         print(f"see near Grid {near_g}")
    #     else:
    #         if len(self.Area_grid[near_g])==0:     #------>  We need update self.Area_grid during state transition!! 
    #             #Change another grid, 
    #             CurGird.remove(near_g)
    #             Diss=[self.Distance(self.wp, (g[0][0]*sr,g[0][1]*sr,0)) for g in CurGrid]
    #             adex=Diss.index(min(Diss)) 
    #             near_g=CurGrid[adex] 
    #     # ########################### Select waypoint within the grid!!!
    #     # Diss=[self.Distance(self.wp, (a[0],a[1],0)) for a in areas]
    #     # adex=Diss.index(min(Diss)) 
    #     # near_a=areas[adex]         # Get the closest task area. 
    #     # to_wp=self.Area_WPC.get(near_a)  # Get the waypoint can cover the area
    #     # ######## Get which mission this area have? 
    #     # for m,st in self.TaskState[near_a].items():
    #     #     #mission,offset=m
    #     #     jr,sr=st
    #     #     if jr==0 and sr==0:
    #     #         #print(f"why fial {m}")
    #     #         mission=(m,GetDL)
    #     # poheights=list(self.drone.WPCinfo.get(mission[0][0]).keys()) 
    #     # wps=[w for w in to_wp if w[2] in poheights] 
    #     # wps.sort(key=lambda s: s[2], reverse=True) # Choose the waypoint height! 
    #     ##################################################################################
    #     ############# Check if wps[0] valid????
    #     Return=self.CheckReturn(wps[0],mission)
    #     #nwpc=wps[0]
    #     if Return:
    #         nwpc=self.ShortUpload.get(self.wp)[1]
    #     else:
    #         nwpc=wps[0]
    #     self.waypointSeq.append(nwpc)
    #     Reward,SReward, Penalty,UpdateGrid=self.ClusterStateTrans(nwpc)
    #     return Reward, SReward, Penalty









