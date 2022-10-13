
import numpy as np
import matplotlib.pyplot as plt
import sys
import geopandas as gpd
from _collections import defaultdict
sys.path.append('../../../')
#from cpp_algorithms.common_helpers import imshow, imshow_scatter
from bcd_helper import imshow, imshow_scatter, cluster_imshow
# from zipfile import ZipFile
#from geopy import distance
#from skimage import measure
import math
import time
import shapely
import shapely.geometry
from collections import defaultdict
from skimage.draw import polygon
import os
import random
from FQ_GenTask import UpadteEFA, logEFA
from FQ_Drones_Info import Sensor, Drone, ReadSen_PPM, LoadDrones
from FQ_Firesim import DyFarsitefile, ClearBarrier, CreateDyRxfire
from FQ_TG_client import GetFirSim, Decomposition
from FQ_Task_Generator import  TaskManager 

import copy


# def Decomposition(DecomposeSize, Res,Task_mission):  # Return the dictionary of grids and area
#     DecomposeSize=int((DecomposeSize/Res)*Res)
#     sc=DecomposeSize/Res
#     AreaPara=defaultdict(dict)
#     p=np.full((len(Task_mission['BM'][1]),len(Task_mission['BM'][1][0])), -1)
#     cc=0
#     #examples: Taskdict['FT']=[init, FTA,Missions['FT'][0],Missions['FT'][1]]
#     tmp=list(Task_mission.keys())[0]
#     perTable=np.full((len(Task_mission[tmp][1]),len(Task_mission[tmp][1][0])), 255)
#     Task_mission=sorted(Task_mission.items(), key=lambda x:x[1][2])
#     #print(f"Check {Task_mission}")
#     #Here is tricky, I sort it by its period, so it grid will not overlapped
#     #Every grid, we only consider the minimum period, right! 
#     cc=0
#     for mm in Task_mission:
#         period=mm[1][2]
#         cov=mm[1][1]
#         Grid_mission=defaultdict(list)
#         Area=defaultdict(list)
#         x,y =np.where(cov==1)
#         for i in range(len(x)):
#             if perTable[x[i],y[i]]>period:
#                 ######### We will prioritize the short period! 
#                 perTable[x[i],y[i]]=period
#                 xc=int(x[i]/sc); yc=int(y[i]/sc)
#                 Area[(xc,yc)].append([x[i],y[i],mm[1][0]])
#                 p[x[i],y[i]]=cc#xc+yc+cc*2
#         AreaPara[mm[0]]=Area
#         cc=cc+1
#     # cluster_imshow(p,sc=sc)
#     # plt.show()
#     return AreaPara

def TrackDraw(Drones,EFAM):
    p=np.full(EFAM.shape, -1)
    for i in range(len(Drones)):
        for g in Drones[i].area:
            grids=g[-3]
            x=[k[0] for k in grids]
            y=[k[1] for k in grids]
            p[x,y]=i        
    imshow(p)
    plt.show()
    return p


def DrawWPCs(Bidders):
    fig=plt.figure()
    ax=plt.axes(projection='3d')
    for i in range(len(Bidders)):
        for m, wpc in Bidders[i].Tak_WPC.items():
            ax.scatter([i[0] for i in wpc], [i[1] for i in wpc],[i[2] for i in wpc])
            seq=Bidders[i].Tak_WSeq[m]
            ax.plot3D([i[0] for i in seq], [i[1] for i in seq],[i[2] for i in seq])
    plt.show()
    


def HisLocDrone(Bidders, GC, GCloc, Grids, sed):
    DNum=len(Bidders)
    dronestate=[(B.id, B.sensortype, B.range/B.speed) for B in Bidders]
    donBM=sorted([i for i in dronestate if i[1]=='RGB'], key=lambda x: x[2]) #long --> short
    # Sort by drone's range
    donF=sorted([i for i in dronestate if i[1]!='RGB'], key=lambda x: x[2])
    def Distance(x):
        return math.sqrt((x[0]-GCloc[0])**2+(x[1]-GCloc[1])**2)
    grids=[(i[0],i[1], Distance(i[-1]),i[-1]) for i in Grids]
    dronestate=[(B.id, B.sensortype, B.range/B.speed) for B in Bidders]
    gridBM=[i for i in grids if i[0]=='BM']
    gridF=[i for i in grids if i[0]!='BM']
    random.seed(sed)
    tmpBM=random.sample(range(len(gridBM)), min(len(donBM),len(gridBM)))
    GridsBM=sorted([gridBM[i] for  i in tmpBM], key=lambda x:x[-2]) # short --> long 
    tmpf=random.sample(range(len(gridF)), min(len(donF),len(gridF)))
    GridsF=sorted([gridF[i] for  i in tmpf], key=lambda x:x[-2]) 
    #print(f"See grids {GridsBM}  {GridsF}")
    loc={}
    for i in range(len(tmpBM)):
        loc[donBM[i][0]]=GridsBM[i]
    for i in range(len(tmpf)):
        loc[donF[i][0]]=GridsF[i]
    return loc

def ExtrmLocDrone(Bidders, GC, GCloc, Grids, sed, Res):
    def Distance(x,y):
        return math.sqrt((x[0]-y[0])**2+(x[1]-y[1])**2)
    DNum=len(Bidders)
    gridBM=[i for i in Grids if i[0]=='BM']
    gridF=[i for i in Grids if i[0]!='BM']
    MaxRange=Res*max([Distance(i[-1],GCloc) for i in Grids])
    #MaxBM=max([Distance(i[-1],GCloc) for i in gridBM])
    dronestate=[(B.id, B.sensortype, max(0,MaxRange-B.range)/B.speed) for B in Bidders] #Smaller is better
    donBM=sorted([i for i in dronestate if i[1]=='RGB'], key=lambda x: x[2], reverse=True) #long --> short
    donF=sorted([i for i in dronestate if i[1]!='RGB'], key=lambda x: x[2],reverse=True)
    asgG=[];loc={}
    asgBM=[]; asgF=[]
    random.seed(sed)
    for i in range(len(donF)):
        if i==0:
            Dis=[Distance(g[-1],GCloc) for g in gridF]
        elif len(donF)>1 and i==1:
            Dis=[-1*Distance(g[-1],GCloc) for g in gridF]
        else:
            Dis=[sum([Distance(g[-1],j[-1]) for j in asgG]) for g in gridF]
        m_d=Dis.index(max(Dis))
        asgG.append(gridF[m_d])
        asgF.append(gridF[m_d])
        gridF.remove(gridF[m_d]) 
    for a in range(len(donBM)):
        if a==0:
            Dis=[Distance(i[-1],GCloc) for i in gridBM]
        else:
            Dis=[sum([Distance(i[-1],j[-1]) for j in asgG]) for i in gridBM]
        m_d=Dis.index(max(Dis))
        loc[donBM[a][0]]=gridBM[m_d]
        asgG.append(gridBM[m_d])
        asgBM.append(gridBM[m_d])
        gridBM.remove(gridBM[m_d])
    #tmpBM=random.sample(range(len(gridBM)), min(len(donBM),len(gridBM)))
    GridsBM=sorted([(i,Distance(i[-1],GCloc ))  for i in asgBM ], key=lambda x:x[-1]) # short --> long 
    GridsF=sorted([(i,Distance(i[-1],GCloc )) for  i in asgF], key=lambda x:x[-1]) 
    #print(GridsBM, GridsF)
    for i in range(len(GridsBM)):
        loc[donBM[i][0]]=GridsBM[i][0]
    for i in range(len(GridsF)):
        loc[donF[i][0]]=GridsF[i][0]
    return loc

def RangeLocDrone(Bidders, GC, GCloc, Grids, sed, Res):
    DNum=len(Bidders)
    dronestate=[(B.id, B.sensortype, B.range/B.speed) for B in Bidders]
    donBM=sorted([i for i in dronestate if i[1]=='RGB'], key=lambda x: x[2]) #long --> short
    donF=sorted([i for i in dronestate if i[1]!='RGB'], key=lambda x: x[2])
    def Distance(x,y):
        return math.sqrt((x[0]-y[0])**2+(x[1]-y[1])**2)
    grids=[(i[0],i[1],i[-1]) for i in Grids]
    gridBM=[i for i in grids if i[0]=='BM']
    gridF=[i for i in grids if i[0]!='BM']
    asgG=[];loc={}
    asgBM=[]; asgF=[]
    random.seed(sed)
    for i in range(len(donF)):
        drone_id=donF[i][0]
        #print(f"see range", Bidders[drone_id].range)
        tmpF=[g for g in gridF if Res*Distance(g[-1],GCloc) <=Bidders[drone_id].range]
        if i==0:
            m_d=random.choice(range(len(tmpF)))
        else:
            Dis=[sum([Distance(i[-1],j[-1]) for j in asgG]) for i in tmpF]
            m_d=Dis.index(max(Dis))
        loc[donF[i][0]]=tmpF[m_d]
        asgG.append(tmpF[m_d])
        asgF.append(tmpF[m_d])
        gridF.remove(tmpF[m_d]) 
    for i in range(len(donBM)):
        drone_id=donBM[i][0]
        #print(f"see range", Bidders[drone_id].range)
        tmpBM=[g for g in gridBM if Res*Distance(g[-1],GCloc) <=Bidders[drone_id].range]
        Dis=[sum([Distance(i[-1],j[-1]) for j in asgG]) for i in tmpBM]
        m_d=Dis.index(max(Dis))
        loc[donBM[i][0]]=tmpBM[m_d]
        asgG.append(tmpBM[m_d])
        asgBM.append(tmpBM[m_d])
        gridBM.remove(tmpBM[m_d])
    #tmpBM=random.sample(range(len(gridBM)), min(len(donBM),len(gridBM)))
    GridsBM=sorted([(i,Distance(i[-1],GCloc ))  for i in asgBM ], key=lambda x:x[-1]) # short --> long 
    GridsF=sorted([(i,Distance(i[-1],GCloc )) for  i in asgF], key=lambda x:x[-1]) 
    for i in range(len(GridsBM)):
        loc[donBM[i][0]]=GridsBM[i][0]
    for i in range(len(GridsF)):
        loc[donF[i][0]]=GridsF[i][0]
    return loc

def RandomLocDrone(Bidders, GC, GCloc, Grids, sed, Res):
    DNum=len(Bidders)
    dronestate=[(B.id, B.sensortype, B.range/B.speed) for B in Bidders]
    donBM=sorted([i for i in dronestate if i[1]=='RGB'], key=lambda x: x[2]) #long --> short
    donF=sorted([i for i in dronestate if i[1]!='RGB'], key=lambda x: x[2])
    def Distance(x,y):
        return math.sqrt((x[0]-y[0])**2+(x[1]-y[1])**2)
    grids=[(i[0],i[1],i[-1]) for i in Grids]
    gridBM=[i for i in grids if i[0]=='BM']
    gridF=[i for i in grids if i[0]!='BM']
    asgG=[];loc={}
    asgBM=[]; asgF=[]
    random.seed(sed)
    for i in range(len(donF)):
        drone_id=donF[i][0]
        #print(f"see range", Bidders[drone_id].range)
        tmpF=[g for g in gridF]# if Res*Distance(g[-1],GCloc) <=Bidders[drone_id].range]
        #if i==0:
        m_d=random.choice(range(len(tmpF)))
        #else:
            #Dis=[sum([Distance(i[-1],j[-1]) for j in asgG]) for i in tmpF]
            #m_d=Dis.index(max(Dis))
        loc[donF[i][0]]=tmpF[m_d]
        asgG.append(tmpF[m_d])
        asgF.append(tmpF[m_d])
        gridF.remove(tmpF[m_d]) 
    for i in range(len(donBM)):
        drone_id=donBM[i][0]
        #print(f"see range", Bidders[drone_id].range)
        tmpBM=[g for g in gridBM]# if Res*Distance(g[-1],GCloc) <=Bidders[drone_id].range]
        #Dis=[sum([Distance(i[-1],j[-1]) for j in asgG]) for i in tmpBM]
       # m_d=Dis.index(max(Dis))
        m_d=random.choice(range(len(tmpBM)))
        loc[donBM[i][0]]=tmpBM[m_d]
        asgG.append(tmpBM[m_d])
        asgBM.append(tmpBM[m_d])
        gridBM.remove(tmpBM[m_d])
    #tmpBM=random.sample(range(len(gridBM)), min(len(donBM),len(gridBM)))
    #GridsBM=sorted([(i,Distance(i[-1],GCloc ))  for i in asgBM ], key=lambda x:x[-1]) # short --> long 
    #GridsF=sorted([(i,Distance(i[-1],GCloc )) for  i in asgF], key=lambda x:x[-1]) 
    for i in range(len(GridsBM)):
        loc[donBM[i][0]]=GridsBM[i][0]
    for i in range(len(GridsF)):
        loc[donF[i][0]]=GridsF[i][0]
    return loc


def RanLocDrone(Bidders, sed, GCloc, Grids):
    DNum=len(Bidders)
    ######## Need to refine the initial location! Finish Initial location 
    #And Prediction distance among grid! 
    #Grids.append([mm, cell, len(grids)*(Res**2),grids,period,(cx,cy)])
    #grids=[i[0] for i in list(AreaPara.keys())]
    grids=[i[1] for i in Grids]
    option=[k for k in grids]
    random.seed(sed)
    tmp=random.sample(range(len(option)), DNum)
    loc=[option[i] for i in tmp]
    return loc

class Bidder:
    def __init__(self, Drone, GCloc, Res,Grids,Missions): # sensor can be: sensor=['RGB', 'THM', 'All']
        self.id=Drone.id
        self.loiter=Drone.loiter
        self.range=Drone.range
        self.sensortype=Drone.sensortype
        self.speed=Drone.speed
        self.FoVinfo=Drone.WPCinfo  # This should be computed by PPM!!!
        self.assignGrid=[]
        self.flytime=0
        self.coverarea=0
        self.area=[]
        self.iniArea=[]
        self.GCloc=(GCloc[0],GCloc[1],0)
        self.Res=Res
        self.DisMission={} # maintain the minimal distance 
        self.DisMax=None
        self.UploadU=0
        self.MaxUpload=0
        self.MissionSet=set()
        self.Tak_WPC=defaultdict(set)
        self.Grid_WPC=defaultdict(set)
        self.PretreatWPC(Drone, Grids)
        self.DisWPCs=defaultdict(dict)
        self.Tak_UT={}
        self.Tak_WSeq={}
        self.Tak_Area=defaultdict(set)
        self.Missions=Missions
        self.InterFly=defaultdict(dict)
    def PretreatWPC(self,Drone, Grids):
        Tak_H_COV={}
        for mm, hcov in Drone.WPCinfo.items():
            tmph=[(h, hcov[h][0]) for h in hcov] 
            if len(tmph)>0:
                maxcov=max([c[1] for c in tmph])
                h=min([h[0] for h in tmph if h[1]==maxcov])
                Tak_H_COV[mm]=(maxcov, h)
            else:
                Tak_H_COV[mm]=(-1, -1)
        for g in Grids:
            m=g[0]
            FOV,H=Tak_H_COV[m]
            if FOV!=-1:
                areas=g[3]
                WPCset=set()
                
                for a in areas:
                    x,y,_=np.array(a)*self.Res
                    WPCset.add(((np.ceil(x/FOV)*FOV+FOV/2),(np.ceil(y/FOV)*FOV+FOV/2),H))
                self.Grid_WPC[g[0], g[1], g[-1]]=WPCset
                #print(f"see {[g[0], g[1], g[-1]]} {self.Grid_WPC[g[0], g[1], g[-1]]}")
            else:
                self.Grid_WPC[g[0], g[1], g[-1]]=set()
                
    # def GetDistance(self,g1,g2):
    #     return math.sqrt((g1[-1][0]-g2[-1][0])**2+(g1[-1][1]-g2[-1][1])**2)
    def GetFlyT(self,w1,w2):
        try:
            ft=self.DisWPCs[w1][w2]
            return ft
        except:
            dis=math.sqrt((w1[0]-w2[0])**2+(w1[1]-w2[1])**2+(w1[2]-w2[2])**2)
            if w1==self.GCloc or w2==self.GCloc:
            #if w2==self.GCloc:
                dis=max(0, dis-self.range)
            ft=dis/self.speed+self.loiter
            self.DisWPCs[w1][w2]=ft
            self.DisWPCs[w2][w1]=ft
            return ft
    
    def NNTime(self,WPCs):
        cw=self.GCloc
        trat=0
        WSeq=[cw]
        wpcs=copy.copy(WPCs)
        while len(wpcs)>0:
            ft=[(self.GetFlyT(cw, w),w) for w in wpcs]
            mint=min([t[0] for t in ft])
            dw=[t[1] for t in ft if t[0]==mint][0]
            trat=trat+mint
            cw=dw
            WSeq.append(cw)
            wpcs.remove(dw)
        trat=trat+self.GetFlyT(cw,self.GCloc)
        WSeq.append(self.GCloc)
        return trat, WSeq
    
    def Get_ImproveUT(self, g):
        m=g[0]
        # if len(self.FoVinfo.get(m))==0:
        #     return 0
        try:
           CurUT=self.Tak_UT[m] 
        except:
            CurUT=0
        WPCs=copy.copy(self.Grid_WPC[g[0],g[1],g[-1]]) # Get WPCs covering the area
        period=g[4]*60
        CurWPCs=set()
        if m in self.Tak_WPC:
            CurWPCs=self.Tak_WPC[m] # Current WPCs
        if len(WPCs-CurWPCs)==0:  # if WPCs contains in current WPCs
            return 0, self.Tak_WSeq[m]
        WPCs.update(CurWPCs)   
        trat, WSeq=self.NNTime(WPCs)
        ut=trat/period
        inUT=ut-CurUT
        return inUT, WSeq # improve UT
####################################################################################  
    def Get_Inter_UT(self, g): # Consider more about inter-mission travel time. 
        m=g[0]
        try:
           CurUT=self.Tak_UT[m] 
        except:
            CurUT=0
        WPCs=copy.copy(self.Grid_WPC[g[0],g[1],g[-1]]) # Get WPCs covering the area
        period=g[4]*60
        CurWPCs=set()
        if m in self.Tak_WPC:
            CurWPCs=self.Tak_WPC[m] # Current WPCs
        if len(WPCs-CurWPCs)==0:  # if WPCs contains in current WPCs
            return 0, 0, {}, self.Tak_WSeq[m]
        AddWP=WPCs-CurWPCs
        WPCs.update(CurWPCs)   
        trat, WSeq=self.NNTime(WPCs)
        ut=trat/period
        inUT=ut-CurUT
        inInter=0
        UpInter=defaultdict(dict)
        ################### Add inter-travel time of m
        if len(self.Tak_WPC)>0:
            CurInter=0
            if len(self.Tak_WPC)==1 and m in self.Tak_WPC:
                return inUT, 0,UpInter, WSeq # improve UT
            if len(self.Tak_WPC)>1 and m in self.InterFly:
                CurInter=sum([v for v in list(self.InterFly[m].values())])
            
            for mm, wp in self.Tak_WPC.items():#Compute the improved intertravel time 
                if mm!=m:
                    pd=self.Missions.get(mm)[0]*60
                    if m in self.InterFly: # Get the minimum distance
                        ITT=min([min([self.GetFlyT(w,ow) for ow in wp]) for w in AddWP])
                        ITT=ITT/(max(pd,period)) # consider the longer period one 
                        ITT=min(ITT,self.InterFly[m][mm]) # Only can reduce
                    else:
                        ITT=min([min([self.GetFlyT(w,ow) for ow in wp]) for w in WPCs])
                        ITT=ITT/(max(pd,period)) # consider the longer period one 
                    #print(f"{m} {mm} {period} {pd}")
                    UpInter[m][mm]=ITT
                    UpInter[mm][m]=ITT
                    inInter=inInter+ITT
            inInter=inInter-CurInter        
        return inUT, inInter,UpInter, WSeq # improve UT
    
    def Get_InterCsot(self, g):
        m=g[0]
        if len(self.FoVinfo.get(m))==0:
            return 10000000000000
        inUT,inInter,_,_=self.Get_Inter_UT(g)
        ToUT=sum(list(self.Tak_UT.values()))+inUT
        ToInter=sum([sum(list(self.InterFly[k].values())) for k in self.InterFly])
        ToInter=ToInter/2+inInter
        return ToUT+ToInter
    
    def UpdateInter(self,g):
        m=g[0]
        _,_,UpInter,_=self.Get_Inter_UT(g)
        if len(UpInter)>0:
            for m1, mt in UpInter.items():
                for m2, t in mt.items():
                    self.InterFly[m1][m2]=t
            #print(f"see Drone {self.id} interfly  {m1} {m2} {t}")
            
    def AddGrid_Inter(self, grid):
        #period=grid[-2]*60
        m=grid[0]
        self.assignGrid.append(grid[1])
        inUT,WSeq=self.Get_ImproveUT(grid)
        if m in self.Tak_UT:    
            self.Tak_UT[m]=self.Tak_UT[m]+inUT
        else:
            self.Tak_UT[m]=inUT
        #############
        self.UpdateInter(grid)
        self.Tak_WPC[m].update(self.Grid_WPC[grid[0],grid[1],grid[-1]]) #Update its WPCs 
        self.Tak_WSeq[m]=WSeq
        #self.Tak_WPC[m]=self.Tak_WPC[m]|self.Grid_WPC[grid[0],grid[1],grid[-1]] #Update its WPCs 
        self.Tak_Area[m].update(set([i[:-1] for i in grid[3]]))
        self.area.append(grid)
        self.MissionSet.add(grid[0])
############################################################################   
    def Get_WPCCsot(self, g):
        m=g[0]
        if len(self.FoVinfo.get(m))==0:
            return 10000000000000
        inUT,_=self.Get_ImproveUT(g)
        ToUT=sum(list(self.Tak_UT.values()))+inUT
        return ToUT
    

    def AddGrid_WPC(self, grid):
        #period=grid[-2]*60
        m=grid[0]
        self.assignGrid.append(grid[1])
        inUT,WSeq=self.Get_ImproveUT(grid)
        if m in self.Tak_UT:    
            self.Tak_UT[m]=self.Tak_UT[m]+inUT
        else:
            self.Tak_UT[m]=inUT
       
        #print(f"Drone {self.id} Add WPCs for {m} improve {inUT} {self.Grid_WPC[grid[0],grid[1],grid[-1]]-self.Tak_WPC[m]}")
        #print(f"Drone {self.id} Add WPCs for {m} improve {inUT} {inUT*period*self.speed}")
        self.Tak_WPC[m].update(self.Grid_WPC[grid[0],grid[1],grid[-1]]) #Update its WPCs 
        self.Tak_WSeq[m]=WSeq
        #self.Tak_WPC[m]=self.Tak_WPC[m]|self.Grid_WPC[grid[0],grid[1],grid[-1]] #Update its WPCs 
        self.Tak_Area[m].update(set([i[:-1] for i in grid[3]]))
        self.area.append(grid)
        self.MissionSet.add(grid[0])
        
    def TraverseSum(self,grid):   # Cell distance
       if len(self.assignGrid)==0:
           return 0#math.sqrt((grid[1][0]-self.iniArea[1][0])**2+(grid[1][1]-self.iniArea[1][1])**2)
       add=sum([self.GetDistance(grid, k) for k in self.area])
       return add   
   
    ############################################### For Area-based task allocation 
    def GetCost_Area(self, grid):
        #print(f"see grid 3 {grid[:3]} {list(self.iniArea)}")

        if len(self.FoVinfo.get(grid[0]))==0:
            return 10000000000000
        FOV=max(list(self.FoVinfo.get(grid[0]).keys()))
        period=grid[-2]*60
        #travel=self.GetTravetime(grid)
        #Back=max(0,self.Res*math.sqrt((grid[-1][0]-self.GCloc[0])**2+(grid[1][1]-self.GCloc[1])**2)-self.range)
        ######### Compute Back travel time! 
        traverse=self.TraverseTime(grid) # minimal distance to assigned grids. 
        travelU=self.flytime+(traverse*self.Res)/(self.speed*period)
        travelU=(traverse*self.Res)/(self.speed*period)
        #UploadU=self.GetUploadTime(grid)
        #UploadU=self.MaxUploadTime(grid)
        # if travel==0:
        #     #print(f"see grid 3 {grid[:3]} {list(self.iniArea)}")
        #     if grid[:2]==list(self.iniArea)[:2]:
        #         return 0
        #     #return 0+self.coverarea+travelU+(grid[2]/(self.speed*FOV*period))+UploadU
        #     #return 0+self.coverarea+travelU#+self.GetUploadTime(None)#+UploadU#+(grid[2]/(self.speed*FOV*period))
        #     #return 0+self.coverarea+self.GetUploadTime(grid)#+UploadU#+(grid[2]/(self.speed*FOV*period))
        #     return self.coverarea+self.GetUploadTime(None)+travelU#UploadU#+(grid[2]/(self.speed*FOV*period))
        #     #return 0+self.coverarea+travelU+self.MaxUploadTime(None)#+(grid[2]/(self.speed*FOV*period))
        l,w=grid[2]
        covt=(math.ceil(l/FOV)*w)/(self.speed*period)
        return self.coverarea+covt#+travelU+self.GetUploadTime(None)#+(grid[2]/(self.speed*FOV*period))#+UploadU#+travel/self.speed
    def AddGrid_Area(self, grid):
        period=grid[-2]*60
        m=grid[0]
        diss=math.sqrt((grid[-1][0]-self.GCloc[0])**2+(grid[1][1]-self.GCloc[1])**2)*self.Res
        if len(self.area)==0:
            self.DisMax=(period, diss, diss)
        else:
            p,maxd,mind=self.DisMax
            self.DisMax=(min(period,p), max(diss, maxd),min(diss, mind))
        if (m, period) in list(self.DisMission.keys()):
            maxd, mind=self.DisMission[m,period]
            self.DisMission[m,period]=(max(maxd,diss), min(mind,diss))
        else:
            self.DisMission[m,period]=(diss, diss)
        FOV=max(list(self.FoVinfo.get(grid[0]).keys()))
        l,w=grid[2]
        covt=(math.ceil(l/FOV)*w)/(self.speed*period)
        self.coverarea=self.coverarea+covt # add area
        #self.flytime=self.flytime+ self.TraverseTime(grid)*self.Res/(self.speed*period)
        # print(f"add grid {grid} iniloc {self.iniArea}")
        # print(f"why flytime {self.TraverseTime(grid)*self.Res} {(self.speed*period)} ")
        self.assignGrid.append(grid[1])
        #self.Tak_WPC[m].update(self.Gird_WPC[(g[0],g[1].g[-1])]) #Update its WPCs 
        self.area.append(grid)
        self.MissionSet.add(grid[0])
        self.UploadU=self.GetUploadTime(None)
        self.Tak_Area[m].update(set([i[:-1] for i in grid[3]]))
        #self.MaxUpload=self.MaxUploadTime(None)
    def GetUploadTime(self,grid):
        UploadU=0
        if grid==None:
            for mm, max_min in self.DisMission.items():
                m, p=mm
                maxd, mind=max_min
                UploadU=UploadU+ (max(0,maxd-self.range)+max(0,mind-self.range))/(self.speed*p)
            return UploadU
        period=grid[-2]*60
        mission=grid[0]
        diss=math.sqrt((grid[-1][0]-self.GCloc[0])**2+(grid[1][1]-self.GCloc[1])**2)*self.Res
        for mm, max_min in self.DisMission.items():
            m, p=mm
            maxd, mind=max_min
            if mission==m:
                UploadU=UploadU+(max(0,max(maxd,diss)-self.range))/(self.speed*p)
                #UploadU=UploadU+(2*max(0,min(mind,diss)-self.range))/(self.speed*p)
            else:
                UploadU=UploadU+(max(0,maxd-self.range))/(self.speed*p)
                #UploadU=UploadU+(2*max(0,mind-self.range))/(self.speed*p)
        if mission not in list(self.DisMission.keys()):
            UploadU=UploadU+ (max(0,diss-self.range))/(self.speed*period)
        return UploadU
    def TraverseTime(self,grid): # Cell Center distance
        #Grids.append([mm, cell, len(grids)*(Res**2),grids,period,(cx,cy)])
        travel=self.GetTravetime(grid)
        if travel==0:
            return 0
        if len(self.assignGrid)==0: 
            return self.GetDistance(grid, self.iniArea)#+ math.sqrt((grid[-1][0]-self.GCloc[0])**2+(grid[1][1]-self.GCloc[1])**2)
        add=min([self.GetDistance(grid, k) for k in self.area])
        #print(f"why distance {add} {[self.GetDistance(grid, k) for k in self.area]}")
        return add
    def GetTravetime(self,grid):   # Cell distance
        if len(self.assignGrid)==0:
            return math.sqrt((grid[1][0]-self.iniArea[1][0])**2+(grid[1][1]-self.iniArea[1][1])**2)
        add=min([math.sqrt((grid[1][0]-k[0])**2+(grid[1][1]-k[1])**2) for k in self.assignGrid])
        return add
    ################################################################ For VD
    def AddGrid_VD(self, grid):
        m=grid[0]
        self.assignGrid.append(grid[1])
        self.area.append(grid)
        self.MissionSet.add(grid[0])
        self.Tak_Area[m].update(set([i[:-1] for i in grid[3]]))

    def GetDistance(self,g1,g2):
            return math.sqrt((g1[-1][0]-g2[-1][0])**2+(g1[-1][1]-g2[-1][1])**2)
    def GetDistance_VD(self,grid):
        if len(self.FoVinfo.get(grid[0]))==0:
            return 10000000000000
        return self.GetDistance(self.iniArea,grid)
        
###################################################  For biddings 
def Bidding_WPC(Grids, Bidders, EFAM):
    while len(Grids)>0: 
        bid=[];costs=[]
        for i in range(len(Bidders)):
            #costs.append([Bidders[i].GetCost(g) for g in Grids])
            #costs.append([Bidders[i].GetCost_New(g) for g in Grids])
            costs.append([Bidders[i].Get_WPCCsot(g) for g in Grids])
           # costs.append([Bidders[i].Get_InterCsot(g) for g in Grids])
            
        bid=[min(k) for k in costs]
        #print(f"see bid {bid}")
        price=min(bid)
        winners=[k for k in range(len(bid)) if bid[k]==price]
        rm=[]
        #print(f"see winner {winners}")
        for j in winners:   # once have multiple winner or one winner????? 
            nums=[k for k in range(len(costs[j])) if costs[j][k]==price and Grids[k] not in rm]
            if len(nums)>1:
                # print(f" multiple Choice ")
                
                srnum=sorted([(i, Bidders[j].TraverseSum(Grids[i])) for i in nums], key=lambda x: x[1])
                #TrackDraw(Bidders,EFAM)
                nums=[s[0] for s in srnum]
                # for a in nums:
                #     print(f'Multipe {Grids[a]}')
            if len(nums)>0:
                Bidders[j].AddGrid_WPC(Grids[nums[0]])
                rm.append(Grids[nums[0]])
            #print(f"see task {Grids[nums[0]]}")
        for r in rm:
            Grids.remove(r)
        #print(f"see Drones flytime {[Bidders[i].flytime for i in range(len(Bidders))]}")
        #print(f"see Drones cost {[sum(list(Bidders[i].Tak_UT.values())) for i in range(len(Bidders))]}")
        # print(f"see Drones upload {[Bidders[i].UploadU for i in range(len(Bidders))]}")
        # print(f"see Drones maxupload {[Bidders[i].MaxUpload for i in range(len(Bidders))]}")
        # print(f"see Drones cost {[Bidders[i].coverarea+Bidders[i].flytime+Bidders[i].MaxUpload for i in range(len(Bidders))]}")
        # print(f"see Drones cost {[Bidders[i].coverarea+Bidders[i].UploadU for i in range(len(Bidders))]}")        #print(f"winner {winners} {len(Grids)}")
        #TrackDraw(Bidders,EFAM)
    return Bidders

def Bidding_Inter(Grids, Bidders, EFAM):
    while len(Grids)>0: 
        bid=[];costs=[]
        for i in range(len(Bidders)):
            #costs.append([Bidders[i].GetCost(g) for g in Grids])
            #costs.append([Bidders[i].GetCost_New(g) for g in Grids])
            #costs.append([Bidders[i].Get_WPCCsot(g) for g in Grids])
            costs.append([Bidders[i].Get_InterCsot(g) for g in Grids])
            
        bid=[min(k) for k in costs]
        #print(f"see bid {bid}")
        price=min(bid)
        winners=[k for k in range(len(bid)) if bid[k]==price]
        rm=[]
        #print(f"see winner {winners}")
        for j in winners:   # once have multiple winner or one winner????? 
            nums=[k for k in range(len(costs[j])) if costs[j][k]==price and Grids[k] not in rm]
            if len(nums)>1:
                # print(f" multiple Choice ")
                
                srnum=sorted([(i, Bidders[j].TraverseSum(Grids[i])) for i in nums], key=lambda x: x[1])
                #TrackDraw(Bidders,EFAM)
                nums=[s[0] for s in srnum]
                # for a in nums:
                #     print(f'Multipe {Grids[a]}')
            if len(nums)>0:
                Bidders[j].AddGrid_Inter(Grids[nums[0]])
                rm.append(Grids[nums[0]])
            #print(f"see task {Grids[nums[0]]}")
        for r in rm:
            Grids.remove(r)
        #print(f"see Drones flytime {[Bidders[i].flytime for i in range(len(Bidders))]}")
        #print(f"see Drones cost {[sum(list(Bidders[i].Tak_UT.values())) for i in range(len(Bidders))]}")
        #print(f" see inter {[0.5*sum([sum(list(Bidders[i].InterFly[k].values())) for k in list(Bidders[i].InterFly)]) for i in range(len(Bidders))]}")
       
    return Bidders

def Bidding_Area(Grids, Bidders, EFAM):
    
    while len(Grids)>0: 
        bid=[];costs=[]
        for i in range(len(Bidders)):
            #costs.append([Bidders[i].GetCost(g) for g in Grids])
            costs.append([Bidders[i].GetCost_Area(g) for g in Grids])
        bid=[min(k) for k in costs]
        price=min(bid)
        winners=[k for k in range(len(bid)) if bid[k]==price]
        rm=[]
        for j in winners:   # once have multiple winner or one winner????? 
            nums=[k for k in range(len(costs[j])) if costs[j][k]==price and Grids[k] not in rm]
            # if len(nums)>1:
            #     srnum=sorted([(i, Bidders[j].TraverseSum(Grids[i])) for i in nums], key=lambda x: x[1])
            #     nums=[s[0] for s in srnum]
            if len(nums)>0:
                Bidders[j].AddGrid_Area(Grids[nums[0]])
                rm.append(Grids[nums[0]])
        for r in rm:
            Grids.remove(r)
        #print(f"see Drones cost {[Bidders[i].coverarea+Bidders[i].UploadU for i in range(len(Bidders))]}")        #print(f"winner {winners} {len(Grids)}")
    return Bidders
    
def Voronoi(Grids, Bidders, EFAM):
    for g in Grids:
        bid=[Bidders[i].GetDistance_VD(g) for i in range(len(Bidders))]
        price=min(bid)
        winners=[k for k in range(len(bid)) if bid[k]==price]
        j=winners[0]
        Bidders[j].AddGrid_VD(g)
    return Bidders

# ####################VD 
# def Voronoi(EFA, DroneNum):    # decomposition using Voronoi method
#     Grid={}
#     Num=DroneNum
#     p=np.full((len(EFA),len(EFA[0])), -1)
#     # suppose 4 groups
#     xn=random.sample(range(len(EFA)), Num)
#     yn=random.sample(range(len(EFA[0])), Num)
#     Group=defaultdict(list) 
#     for i in range(Num):
#         Group[i].append((xn[i],yn[i]))
#     for i in range(len(EFA)):
#         for j in range(len(EFA[i])):
#             distances=[(i-xn[k])**2+(j-yn[k])**2 for k in range(Num)]
#             C= distances.index(min(distances))
#             p[i][j]=C
    # imshow(p)
    # #plt.scatter(xn,yn, color='black')
    # plt.show()
    # return p, xn,yn

def Auction_Comp(which, AreaPara, Drones, Res,Missions, seed, EFAM, GCloc):
    #GCloc=np.array(GCloc)//Res
    Grids=[] # Get all grids
    FGrids=[]
    AGrids=[]
    for key, grids in AreaPara.items():
        mm=key[1]
        cell=key[0]
        period=Missions.get(mm)[0]
        x=[i[0] for i in grids]
        y=[i[1] for i in grids]
        cx=(max(x)+min(x))//2
        cy=(max(y)+min(y))//2
        length=(max(x)-min(x)+1)*Res
        width=(max(y)-min(y)+1)*Res
        Grids.append([mm, cell,(length, width),grids,period,(cx,cy)])
        if mm=='BM':
            AGrids.append([mm, cell, (length, width),grids,period,(cx,cy)])
        else:
            FGrids.append([mm, cell, (length, width),grids,period,(cx,cy)])
    #loc=HisLocDrone(Bidders,seed, GCloc, Grids, seed)
    Bidders=[]
    for d in Drones:
        #print(f"see wpcinfo {d.WPCinfo}")
        Bidders.append(Bidder(d,GCloc, Res,Grids,Missions))
    gcloc=np.array(GCloc)//Res
    loc=ExtrmLocDrone(Bidders, seed, gcloc, Grids, seed, Res)
    
    #Bidders=Bidding(Grids,Bidders, EFAM)
    #############################
    #########Fire first, then others
    if which==2: # Inter 
        for i in range(len(Bidders)):
            Bidders[i].AddGrid_Inter(loc[i])
        Bidders=Bidding_Inter(FGrids,Bidders, EFAM)
        #TrackDraw(Bidders,EFAM)
        #DrawWPCs(Bidders)
        Bidders=Bidding_Inter(AGrids,Bidders, EFAM)
    if which==1: # Only WPC
    #########Fire first, then others
        for i in range(len(Bidders)):
            Bidders[i].AddGrid_WPC(loc[i])
        Bidders=Bidding_WPC(FGrids,Bidders, EFAM)
        #TrackDraw(Bidders,EFAM)
        #DrawWPCs(Bidders)
        Bidders=Bidding_WPC(AGrids,Bidders, EFAM)
    if which==3: # Area_based 
        for i in range(len(Bidders)):
            Bidders[i].AddGrid_Area(loc[i])
            Bidders[i].iniArea=loc[i] ########### Add it for VD
        Bidders=Bidding_Area(FGrids,Bidders, EFAM)
        #TrackDraw(Bidders,EFAM)
        #DrawWPCs(Bidders)
        Bidders=Bidding_Area(AGrids,Bidders, EFAM)
    if which==4: # Run Voronoi 
        #loc=ExtrmLocDrone(Bidders,seed, gcloc, Grids, seed, Res)
        loc=RandomLocDrone(Bidders, seed, gcloc, Grids, seed, Res)
        for i in range(len(Bidders)):
            Bidders[i].AddGrid_VD(loc[i])
            Bidders[i].iniArea=loc[i] ########### Add it for VD
        Bidders=Voronoi(Grids, Bidders, EFAM)
        #TrackDraw(Bidders,EFAM)
    for i in range(len(Bidders)):
        #Bidders[i].CombineGrid()
        Drones[i].area=Bidders[i].area
        Drones[i].MissionSet=Bidders[i].MissionSet
        Drones[i].Tak_Area=Bidders[i].Tak_Area
        #print(f"check what is area {Drones[i].area}")
    #DrawWPCs(Bidders)
    return Drones, Bidders


IFLOG=False
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
    logfile=f"logtask.txt"
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
    #CreateDyRxfire(Bardata, Bursite,'FQ_burn',dir, [2])
    fir_name='FQ_Sim_10_5'
    EFA,EFAdict,Primdict =GetFirSim(Bardata,  foldername, fir_name, dir, Bursite, Res=Res,time=1)                  
    TM=TaskManager(Missions)
    cut=time.time()
    print(f"Start Task Generation")
    tasks=TM.DeclareGridEFA(EFA,init=0)
    print(f" task generation tasks {time.time()-cut}")
    ########################################
    sensorfile='Data/sensor_info.csv'
    PPMfile='Data/PPM_table.csv'
    # DroneNum=5
    # speeds=[5,5,5,5,3,3]
    # loiter=[1,2,1,1,1]
    # ranges=[300,200,500,300,300]
    DroneNum=6
    speeds=[5,5,5,5,4,5]
    loiter=[1,2,1,1,1,2]
    ranges=[300,200,500,300,300,500]
    #ranges=[200,100,100,300,100]
    GCloc=(0,500)
    GCloc=(0,0)
    inloc=(0,0,0)
    sensgrp=[['ZENMUSE_XT2_t','ZENMUSE_XT2_r'],['DJI_Air2S'],['ZENMUSE_XT2_t','ZENMUSE_XT2_r'],['DJI_Air2S'],['ZENMUSE_XT2_t','ZENMUSE_XT2_r'],['ZENMUSE_XT2_t','ZENMUSE_XT2_r']]
    #Drones=LoadDrones(sensorfile,PPMfile,DroneNum, speeds, sensgrp, Res,loiter)
    DrawTask(tasks,EFA) 
    #Voronoi(EFA,DroneNum)
    Drones=LoadDrones(sensorfile,PPMfile,DroneNum, speeds, sensgrp, Res,loiter,ranges)
    
    #Task_mission,BSshp=GenTasks(init,end,Bursite,dir,foldername,Missions, Res=Res)
    ########################## Do decomposition~  1: Normal 2: Inter 3: Area 4: Voronoi
    AreaPara=Decomposition(DecomposeSize,Res,tasks) # Need put tasks there!!!
    which=0
    Drones,Bidders=Auction_Comp(which, AreaPara, Drones, Res,Missions,3, EFA,GCloc)
    #print(f"see Drones cost {[sum(list(Bidders[i].Tak_UT.values())) for i in range(len(Bidders))]}")
    TrackDraw(Drones, EFA)


    # def GetTravetime(self,grid):   # Cell distance
    #     if len(self.assignGrid)==0:
    #         return math.sqrt((grid[1][0]-self.iniArea[1][0])**2+(grid[1][1]-self.iniArea[1][1])**2)
    #     add=min([math.sqrt((grid[1][0]-k[0])**2+(grid[1][1]-k[1])**2) for k in self.assignGrid])
    #     return add
   
    # def TraverseTime(self,grid): # Cell Center distance
    #     #Grids.append([mm, cell, len(grids)*(Res**2),grids,period,(cx,cy)])
    #     travel=self.GetTravetime(grid)
    #     if travel==0:
    #         return 0
    #     if len(self.assignGrid)==0: 
    #         return self.GetDistance(grid, self.iniArea)#+ math.sqrt((grid[-1][0]-self.GCloc[0])**2+(grid[1][1]-self.GCloc[1])**2)
    #     add=min([self.GetDistance(grid, k) for k in self.area])
    #     #print(f"why distance {add} {[self.GetDistance(grid, k) for k in self.area]}")
    #     return add
    
    # def GetUploadTime(self,grid):
    #     UploadU=0
    #     if grid==None:
    #         for mm, max_min in self.DisMission.items():
    #             m, p=mm
    #             maxd, mind=max_min
    #             UploadU=UploadU+ (max(0,maxd-self.range)+max(0,mind-self.range))/(self.speed*p)
    #         return UploadU
    #     period=grid[-2]*60
    #     mission=grid[0]
    #     diss=math.sqrt((grid[-1][0]-self.GCloc[0])**2+(grid[1][1]-self.GCloc[1])**2)*self.Res
    #     for mm, max_min in self.DisMission.items():
    #         m, p=mm
    #         maxd, mind=max_min
    #         if mission==m:
    #             UploadU=UploadU+(max(0,max(maxd,diss)-self.range))/(self.speed*p)
    #             #UploadU=UploadU+(2*max(0,min(mind,diss)-self.range))/(self.speed*p)
    #         else:
    #             UploadU=UploadU+(max(0,maxd-self.range))/(self.speed*p)
    #             #UploadU=UploadU+(2*max(0,mind-self.range))/(self.speed*p)
    #     if mission not in list(self.DisMission.keys()):
    #         UploadU=UploadU+ (max(0,diss-self.range))/(self.speed*period)
    #     return UploadU
    
    # def MaxUploadTime(self,grid):
    #     if grid==None:
    #         try:
    #             minp, maxd,mind=self.DisMax
    #             UploadU= (max(0,maxd-self.range)+max(0,mind-self.range))/(self.speed*minp)
    #             return UploadU
    #         except:
    #             return 0
    #     period=grid[-2]*60
    #     diss=math.sqrt((grid[-1][0]-self.GCloc[0])**2+(grid[1][1]-self.GCloc[1])**2)*self.Res
    #     if len(self.area)>0:
    #         minp, maxd,mind=self.DisMax
    #         period=min(minp,period);maxd=max(diss,maxd);mind=min(diss,mind)
    #     else:
    #         period=period; maxd=diss; mind=diss
    #     UploadU=(max(0,maxd-self.range)+max(0,mind-self.range))/(self.speed*period)
    #     return UploadU

    # def AddGrid(self, grid):
    #     period=grid[-2]*60
    #     m=grid[0]
    #     diss=math.sqrt((grid[-1][0]-self.GCloc[0])**2+(grid[1][1]-self.GCloc[1])**2)*self.Res
    #     if len(self.area)==0:
    #         self.DisMax=(period, diss, diss)
    #     else:
    #         p,maxd,mind=self.DisMax
    #         self.DisMax=(min(period,p), max(diss, maxd),min(diss, mind))
    #     if (m, period) in list(self.DisMission.keys()):
    #         maxd, mind=self.DisMission[m,period]
    #         self.DisMission[m,period]=(max(maxd,diss), min(mind,diss))
    #     else:
    #         self.DisMission[m,period]=(diss, diss)
    #     FOV=max(list(self.FoVinfo.get(grid[0]).keys()))
    #     l,w=grid[2]
    #     covt=(math.ceil(l/FOV)*w)/(self.speed*period)
    #     self.coverarea=self.coverarea+covt # add area
    #     self.flytime=self.flytime+ self.TraverseTime(grid)*self.Res/(self.speed*period)
    #     print(f"add grid {grid} iniloc {self.iniArea}")
    #     print(f"why flytime {self.TraverseTime(grid)*self.Res} {(self.speed*period)} ")
    #     self.assignGrid.append(grid[1])
    #     #print(f"see {self.Grid_WPC} {grid[0],grid[1],grid[-1]}")
    #     print(self.Grid_WPC[grid[0],grid[1],grid[-1]])
    #     self.Tak_WPC[m].update(self.Grid_WPC[grid[0],grid[1],grid[-1]]) #Update its WPCs 
    #
    #     self.area.append(grid)
    #     self.MissionSet.add(grid[0])
    #     self.UploadU=self.GetUploadTime(None)
    #     self.MaxUpload=self.MaxUploadTime(None)











