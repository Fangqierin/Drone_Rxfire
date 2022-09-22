import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from bcd_helper import imshow, imshow_EFA, imshow_Task
import math
from collections import defaultdict
import random
import time
import geopandas as gpd
from FQ_TaskAllocation_WPC import  Auction, TrackDraw
from FQ_Drones_Info import Sensor, Drone, ReadSen_PPM,LoadDrones
from FQ_Task_Generator import  TaskManager 
from FQ_Firesim import DyFarsitefile, ClearBarrier, CreateDyRxfire
from FQ_TG_client import GetFirSim, Decomposition, DrawTask
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
    Bursite=(702460.0,4309700.0,702860.0,4310200.0 )
    Res=10
    #########We should meet sometimes,################# Do decomposition~ 
    file='CARB_BurnUnits/CARB_BurnUnits.shp'
    data=gpd.read_file(f"{dir}/{file}")
    Bardata=ClearBarrier(data)
    fir_name='FQ_Sim_15_5'
    #CreateDyRxfire(Bardata,fir_name,dir, [2])
    EFA,EFAdict,Primdict =GetFirSim(Bardata,  foldername, fir_name, dir, Bursite, Res=Res,time=20)                  
    imshow_EFA(EFA)
    TM=TaskManager(Missions)
    tasks=TM.DeclareGridEFA(EFA,init=0)
    print(f"check task {tasks}")
    DrawTask(tasks,EFA) 























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









