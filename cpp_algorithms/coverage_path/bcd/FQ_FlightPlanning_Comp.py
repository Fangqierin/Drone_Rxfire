import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from bcd_helper import imshow, imshow_scatter,imshow_EFA,DrawTask
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
from FQ_TaskAllocation_Comp import Auction_Comp
import copy
class TaskState:
    def __init__(self, mission, grid, init, period): #init_time should be eclpse time!
        self.mission=mission
        self.grid=grid
        self.initime=init 
        self.period=period
        self.deadline= 0
        self.JR=0
        self.SR=0
        self.CP=0
    def UpState(self):
        pass
# we need know its flying height, sensor coverage! cover reward!!!!!!!!!!!
class FlightPlanner:
    def __init__(self, Drone, init=0, iniloc=[], GCloc=[], endloc=[], Missions={}, Res=10):
        self.drone=Drone
        self.tasks=Drone.area
        self.GCloc=GCloc
        self.Res=Res
        self.MissionDict=Missions
        self.cw=0
        self.time=init
        self.EndTime=init
        self.wp=iniloc
        self.endloc=endloc
        self.TaskState=defaultdict(dict)# Deadline, Reward, stored reward, Acum. Penalty
        self.WPCHeight=defaultdict(set)
        self.WPCs=[iniloc]
        self.ShortUpload={}
        self.ShortUpload[iniloc]=(self.drone.loiter,iniloc)
        self.Cover_map=defaultdict(list)
        self.Area_WPC=defaultdict(list)
        self.Connection={} #should be waypoint:0/1 yes or no has network!!!
        self.Con_wpc=[]
        self.sigma=10
        self.waypointSeq=[]
        self.Mission_Area=defaultdict(list)
        self.Mission_JR=defaultdict(dict)
        self.Mission_SR=defaultdict(dict)
        self.Mission_DL={}
        self.DisMatrix=defaultdict(dict)
        self.Area_grid={} # for mapping area to grid! {area: (grid, mission)}
        self.Grid_Area=defaultdict(list)# {(mission, offset, grid): [area]} 
        self.Grid_toCov=defaultdict(list) # for a grid, some area need be covered
        self.Fix_toCov=defaultdict(list)
        self.Mission_Grid=defaultdict(set) #{(mission, offset):{grid}}
        self.Area_Task=defaultdict(list)
        self.Area_Mission_time=defaultdict(dict)
        self.WPC_TakFov=defaultdict(dict)
        #Then I need to the the location of the GC!!!
        self.GridMatrix=defaultdict(dict)
        self.logfile=[]
        self.logtasks=defaultdict(dict)
        self.logmiss=defaultdict(dict)
        self.logReward={}
    def GenWaypoint_SetCover(self):
        allh=[]
        for tak in list(self.drone.MissionSet):
            heights=self.drone.WPCinfo.get(tak)
            tmph=[[h, heights[h][0]] for h in heights]
            tmph=sorted(tmph, key=lambda l:l[0], reverse=False) 
            tst=[];delset=[]
            for t in tmph:
                if t[1] not in tst:
                    tst.append(t[1])
                else:   ############ It is okay, because when I combine that, I guarantee h<<, cov<<
                    delset.append(t)
            for d in delset:
                tmph.remove(d)
            allh=allh+[i[0] for i in tmph] ## Get all heights for Grids. 
            self.WPC_TakFov[tak]=tmph
        x0,y0,z=np.array(self.GCloc)
        px=[i for i in range(x0-self.drone.range//self.Res,x0+self.drone.range//self.Res) if i>=0 and i%(2)==0]
        allz= list(range(int(min(allh)), int(max(allh)),self.Res*2))+[ int(max(allh))]
        yrange=[y0-self.drone.range, y0+self.drone.range ]
        upwpc=set()
        for x in px:
            for z in allz:
                tmp=(self.drone.range)**2-z**2-((x-x0)*self.Res)**2
                if tmp>=0:
                    y1=(np.sqrt(tmp))//self.Res+y0
                    y2= y0-(np.sqrt(tmp))//self.Res 
                    upwpc.add((x,int(y1),z))
                    upwpc.add((x,int(y2),z))
        UW=list(upwpc)
        ##########################################################
        for grid in self.tasks:#Get the minimum and maximum x
            areas=grid[3]
            tak=grid[0]
            for a in areas:
                self.Area_Task[a[0], a[1]].append((tak,a[2]))
                self.Area_grid[a[0], a[1]]=(0,0)#grid[1])   #################### FQFQ update
                #self.Area_grid[a[0], a[1]]=(grid[1])   #################### FQFQ update
            xal=[x[0] for x in areas]
            yal=[x[1] for x in areas]
            tmph=self.WPC_TakFov[tak]
            for wpc in tmph:
                fov=wpc[1]/(self.Res)####### Already divided by Res!!!
                mx=math.ceil(min(xal)/fov)*fov   
                allx=list(np.arange(mx, max(xal)+1,fov))
                if min(xal)<mx:
                    allx=[min(xal)]+allx
                if len(allx)==0:
                    allx=[min(xal)]
                # if allx[-1]<max(xal)-fov:
                #     allx=allx+[max(xal)-fov]
                # if fov==13:
                #     print(f"why x {fov} {(min(xal), max(xal)) } {allx}")
                for x in allx:#[min(xal)]+tmp+[max(xal)-fov]:
                    my=math.ceil(min(yal)/fov)*fov   
                    ally=list(np.arange(my, max(yal)+1,fov))
                    if min(yal)<my:
                        ally=[min(yal)]+ally
                    if len(ally)==0:
                        ally=[min(yal)]
                    # if ally[-1]<max(yal)-fov:
                    #     ally=ally+[max(yal)-fov]
                    # if fov==13:
                    #     print(f"why y {fov} {(min(yal), max(yal)) }  {ally}")
                    for y in ally:
                        self.WPCHeight[tak,wpc[0]].add((x+fov/2,y+fov/2,wpc[0]))    
                        #self.WPCs.append((x+fov/2,y+fov/2,wpc[0]))
                        conn=(self.Distance((x+fov/2,y+fov/2,wpc[0]), self.GCloc)<=self.drone.range)
                        self.Connection[x+fov/2,y+fov/2,wpc[0]]=conn
                        ################################### NO Connection! 
                        if conn==True:
                            self.Con_wpc.append((x+fov/2,y+fov/2,wpc[0]))
                            self.ShortUpload[(x+fov/2,y+fov/2,wpc[0])]=(self.drone.loiter,(x+fov/2,y+fov/2,wpc[0])) # if is connected, it is itself. 
                        else:
                            Dis=[self.Distance((x+fov/2,y+fov/2,wpc[0]), ww) for ww in UW]
                            wm=Dis.index(min(Dis))
                            Ttime=min(Dis)/self.drone.speed+self.drone.loiter # maybe 1 s
                            self.ShortUpload[(x+fov/2,y+fov/2,wpc[0])]=(Ttime,UW[wm])
                            self.WPCs.append(UW[wm])
                            self.Connection[UW[wm]]=True
                            self.ShortUpload[UW[wm]]=(self.drone.loiter,UW[wm])
                        ########################### For get the coverage area
                        #if self.Cover_map.get((x+fov/2,y+fov/2,wpc[0]))==None:
                        cx=int(x)
                        while cx<x+fov:
                           cy=int(y) 
                           while cy<y+fov:
                                if self.Cover_map.get((x+fov/2,y+fov/2,wpc[0]))==None:
                                    self.Cover_map[x+fov/2,y+fov/2,wpc[0]].append((cx,cy,tak))
                                    self.Area_WPC[(cx,cy),tak].append((x+fov/2,y+fov/2,wpc[0]))
                                elif (cx,cy,tak) not in self.Cover_map[x+fov/2,y+fov/2,wpc[0]]:
                                    self.Cover_map[x+fov/2,y+fov/2,wpc[0]].append((cx,cy,tak))
                                    self.Area_WPC[(cx,cy),tak].append((x+fov/2,y+fov/2,wpc[0]))
                                cy=cy+1
                           cx=cx+1
                        ########################
        self.SetCoveWPC()
    def SetCoveWPC(self):
        for key, wpc in self.WPCHeight.items():
            mm,h=key
            
            #sorted(list(wpc))
            wps = sorted(list(wpc), key=lambda x: x[0])
            tmph=self.WPC_TakFov[mm]
            for wpc in tmph:
                if wpc[0]==h:
                    fov=wpc[1]/(self.Res)####### Already divided by Res!!!
            cuwpcs=[wps[0]]
            #cuwpcs=[]
            alltak=copy.copy(self.drone.Tak_Area[mm])
            tmparea=alltak-set([a[:-1] for a in self.Cover_map[wps[0]]])
            # if h==120:
            #     for w in wps:
            #         print(w)
            #     fig=plt.figure()
            #     ax=plt.axes(projection='3d')
            #     ax.scatter([i[0]-fov/2 for i in wps],[i[1]-fov/2 for i in wps],[i[2] for i in wps] )
            #     plt.title(f" initIal WPS {mm} {h}")
            #     plt.show()
            wps.remove(wps[0])
            # Allx=set([w[0] for w in wps])
            # Ally=set(w[1] for w in wps)
            
            while len(tmparea)>0:
                #print([[a[:-1] for a in self.Cover_map[w]] for w in wps])
                cov=[(set([a[:-1] for a in self.Cover_map[w]]).intersection(tmparea),w) for w in wps]
                #print(f"fail1", cov)
                #print(f"check {[len(c[0]) for c in cov]} {cov} {tmparea}")
                MaxLen=max([len(c[0]) for c in cov])
                sotcov=[c for c in cov if len(c[0])==MaxLen]
                #sotcov=sorted([c for c in longcov], key=lambda x: x[1][1]/fov)
                sotcov=sorted([c for c in  sotcov], key=lambda x: x[1][0])
                cuwpcs.append(sotcov[0][1])
                wps.remove(sotcov[0][1])
                tmparea=tmparea-sotcov[0][0]
            self.WPCs=self.WPCs+cuwpcs 
            # fig=plt.figure()
            # ax=plt.axes(projection='3d')
            # ax.scatter([i[0] for i in cuwpcs],[i[1] for i in cuwpcs],[i[2] for i in cuwpcs] )
            # plt.title(f"{mm} {h}")
            # plt.show()
        st=time.time()
        self.WPCs=list(set(self.WPCs))
        for i in range(len(self.WPCs)):
            for j in range(i+1,len(self.WPCs)):
                self.DisMatrix[self.WPCs[i]][self.WPCs[j]]=self.Distance(self.WPCs[i], self.WPCs[j])
                self.DisMatrix[self.WPCs[j]][self.WPCs[i]]=self.DisMatrix[self.WPCs[i]][self.WPCs[j]]
        print(f"time to compute distance {time.time()-st}")
    def GenWaypoint_Regular(self):
        Tak_H_FOV={}
        allh=[]
        for tak in list(self.drone.MissionSet):
            heights=self.drone.WPCinfo.get(tak)
            tmph=[[h, heights[h][0]] for h in heights]
            tmph=sorted(tmph, key=lambda l:l[0], reverse=False) 
            tst=[];delset=[]
            for t in tmph:
                if t[1] not in tst:
                    tst.append(t[1])
                else:   ############ It is okay, because when I combine that, I guarantee h<<, cov<<
                    delset.append(t)
            for d in delset:
                tmph.remove(d)
            allh=allh+[i[0] for i in tmph] ## Get all heights for Grids. 
            Tak_H_FOV[tak]=tmph
            self.WPC_TakFov[tak]=tmph
        x0,y0,z=np.array(self.GCloc)
        px=[i for i in range(x0-self.drone.range//self.Res,x0+self.drone.range//self.Res) if i>=0 and i%(2)==0]
        allz= list(range(int(min(allh)), int(max(allh)),self.Res*2))+[ int(max(allh))]
        yrange=[y0-self.drone.range, y0+self.drone.range ]
        upwpc=set()
        for x in px:
            for z in allz:
                tmp=(self.drone.range)**2-z**2-((x-x0)*self.Res)**2
                if tmp>=0:
                    y1=(np.sqrt(tmp))//self.Res+y0
                    y2= y0-(np.sqrt(tmp))//self.Res 
                    upwpc.add((x,int(y1),z))
                    upwpc.add((x,int(y2),z))
        UW=list(upwpc)
        ##########################################################
        for grid in self.tasks:#Get the minimum and maximum x
            areas=grid[3]
            tak=grid[0]
            for a in areas:
                self.Area_Task[a[0], a[1]].append((tak,a[2]))
                self.Area_grid[a[0], a[1]]=(0,0)#grid[1])   #################### FQFQ update
                #self.Area_grid[a[0], a[1]]=(grid[1])   #################### FQFQ update
            xal=[x[0] for x in areas]
            yal=[x[1] for x in areas]
            tmph=Tak_H_FOV[tak]
            for wpc in tmph:
                fov=wpc[1]/(self.Res)####### Already divided by Res!!!
                x=math.floor(min(xal)/fov)*fov  
                while x<=math.ceil(max(xal)/fov)*fov: 
                    y=math.floor(min(yal)/fov)*fov
                    while y<=math.ceil(max(yal)/fov)*fov: 
                        self.WPCHeight[tak,wpc[0]].add((x+fov/2,y+fov/2,wpc[0]))    
                        self.WPCs.append((x+fov/2,y+fov/2,wpc[0]))
                        conn=(self.Distance((x+fov/2,y+fov/2,wpc[0]), self.GCloc)<=self.drone.range)
                        self.Connection[x+fov/2,y+fov/2,wpc[0]]=conn
                        ################################### NO Connection! 
                        if conn==True:
                            self.Con_wpc.append((x+fov/2,y+fov/2,wpc[0]))
                            self.ShortUpload[(x+fov/2,y+fov/2,wpc[0])]=(self.drone.loiter,(x+fov/2,y+fov/2,wpc[0])) # if is connected, it is itself. 
                        else:
                            Dis=[self.Distance((x+fov/2,y+fov/2,wpc[0]), ww) for ww in UW]
                            wm=Dis.index(min(Dis))
                            Ttime=min(Dis)/self.drone.speed+self.drone.loiter # maybe 1 s
                            self.ShortUpload[(x+fov/2,y+fov/2,wpc[0])]=(Ttime,UW[wm])
                            self.WPCs.append(UW[wm])
                            self.Connection[UW[wm]]=True
                            self.ShortUpload[UW[wm]]=(self.drone.loiter,UW[wm])
                            #UsedUW.add(UW[wm])
                        ########################### For get the coverage area
                        cx=int(x)
                        while cx<x+fov:
                           cy=int(y) 
                           while cy<y+fov:
                                if self.Cover_map.get((x+fov/2,y+fov/2,wpc[0]))==None:
                                    self.Cover_map[x+fov/2,y+fov/2,wpc[0]].append((cx,cy,tak))
                                    self.Area_WPC[(cx,cy),tak].append((x+fov/2,y+fov/2,wpc[0]))
                                elif (cx,cy,tak) not in self.Cover_map[x+fov/2,y+fov/2,wpc[0]]:
                                    self.Cover_map[x+fov/2,y+fov/2,wpc[0]].append((cx,cy,tak))
                                    self.Area_WPC[(cx,cy),tak].append((x+fov/2,y+fov/2,wpc[0]))
                                cy=cy+1
                           cx=cx+1
                        ########################
                        y=y+fov
                    x=x+fov
        st=time.time()
        self.WPCs=list(set(self.WPCs))
        for i in range(len(self.WPCs)):
            for j in range(i+1,len(self.WPCs)):
                self.DisMatrix[self.WPCs[i]][self.WPCs[j]]=self.Distance(self.WPCs[i], self.WPCs[j])
                self.DisMatrix[self.WPCs[j]][self.WPCs[i]]=self.DisMatrix[self.WPCs[i]][self.WPCs[j]]
        print(f"time to compute distance {time.time()-st}")
        
    def GetAllGird(self):
        #GridMatrix=defaultdict(dict)
        for grid in self.tasks:#Get the minimum and maximum x
            areas=grid[3]
            tak=grid[0]
            #print(grid)
            xal=[x[0] for x in areas]
            yal=[x[1] for x in areas]
            if self.GridMatrix[tak].get(grid[1])!=None:
                mx,ax,my,ay=self.GridMatrix[tak][grid[1]]
                self.GridMatrix[tak][grid[1]]=(min(xal+[mx]), max(xal+[ax]), min(yal+[my]), max(yal+[ay]))
            else:  
                self.GridMatrix[tak][grid[1]]=(min(xal), max(xal), min(yal), max(yal))
            
    def CheckBoundary(self,grid):
        gx,gy=grid[1]
        tak=grid[0]
        XB=True; YB=True
        myx,ayx,myy,ayy=self.GridMatrix[tak].get(grid[1])
        #check x and check y
        if self.GridMatrix[tak].get((gx-1,gy))!=None:
            mx,ax,my,ay=self.GridMatrix[tak].get((gx-1,gy))
        #print(f"why {ax} {myx+1}")                            
            if ax==myx-1:
                XB=False
        if self.GridMatrix[tak].get((gx,gy-1))!=None:
            mx,ax,my,ay=self.GridMatrix[tak].get((gx,gy-1))
            if ay==myy-1:
                YB=False
        #print(f"check grid {grid[1]} {self.GridMatrix[tak].get(grid[1])}  {self.GridMatrix[tak].get((gx-1,gy))} {XB} {YB}")
        return XB,YB
    
    def GenWaypoint_Adjust(self):
        self.GetAllGird()
        allh=[]
        for tak in list(self.drone.MissionSet):
            heights=self.drone.WPCinfo.get(tak)
            tmph=[[h, heights[h][0]] for h in heights]
            tmph=sorted(tmph, key=lambda l:l[0], reverse=False) 
            tst=[];delset=[]
            for t in tmph:
                if t[1] not in tst:
                    tst.append(t[1])
                else:   ############ It is okay, because when I combine that, I guarantee h<<, cov<<
                    delset.append(t)
            for d in delset:
                tmph.remove(d)
            allh=allh+[i[0] for i in tmph] ## Get all heights for Grids. 
            self.WPC_TakFov[tak]=tmph
        x0,y0,z=np.array(self.GCloc)
        px=[i for i in range(x0-self.drone.range//self.Res,x0+self.drone.range//self.Res) if i>=0 and i%(2)==0]
        allz= list(range(int(min(allh)), int(max(allh)),self.Res*2))+[ int(max(allh))]
        yrange=[y0-self.drone.range, y0+self.drone.range ]
        upwpc=set()
        for x in px:
            for z in allz:
                tmp=(self.drone.range)**2-z**2-((x-x0)*self.Res)**2
                if tmp>=0:
                    y1=(np.sqrt(tmp))//self.Res+y0
                    y2= y0-(np.sqrt(tmp))//self.Res 
                    upwpc.add((x,int(y1),z))
                    upwpc.add((x,int(y2),z))
        UW=list(upwpc)
        ##########################################################
        for grid in self.tasks:#Get the minimum and maximum x
            XB,YB=self.CheckBoundary(grid)
            areas=grid[3]
            tak=grid[0]
            for a in areas:
                self.Area_Task[a[0], a[1]].append((tak,a[2]))
                self.Area_grid[a[0], a[1]]=(0,0)#grid[1])   #################### FQFQ update
                #self.Area_grid[a[0], a[1]]=(grid[1])   #################### FQFQ update
            xal=[x[0] for x in areas]
            yal=[x[1] for x in areas]
            tmph=self.WPC_TakFov[tak]
            for wpc in tmph:
                fov=wpc[1]/(self.Res)####### Already divided by Res!!!
                # mx=math.ceil(min(xal)/fov)*fov   
                # allx=list(np.arange(mx, max(xal),fov))
                if XB:
                    mx=math.ceil(min(xal)/fov)*fov   
                    allx=list(np.arange(mx, max(xal)+1,fov))
                    if min(xal)<mx:
                        allx=[min(xal)]+allx
                    if len(allx)==0:
                        allx=[min(xal)]
                else:
                    mx=math.floor(min(xal)/fov)*fov   
                    allx=list(np.arange(mx, max(xal)+1,fov))
                    # if np.mod((min(xal)-1),fov)!=0:
                    #     allx=[math.floor(min(xal)/fov)*fov]+allx
                    # else:
                    #     allx=[min(xal)]+allx

                    if len(allx)==0:
                        allx=[min(xal)]
                # if allx[-1]<max(xal)-fov:
                #     allx=allx+[max(xal)-fov]
                # if fov==13:
                #     print(f"why x {fov} {(min(xal), max(xal)) } {np.array(allx)}")
                    
                for x in allx:#[min(xal)]+tmp+[max(xal)-fov]: 
                    if YB:
                        my=math.ceil(min(yal)/fov)*fov   
                        ally=list(np.arange(my, max(yal)+1,fov))
                        if min(yal)<my:
                            ally=[min(yal)]+ally
                        if len(ally)==0:
                            ally=[min(yal)]
                    else:
                        my=math.floor(min(yal)/fov)*fov   
                        ally=list(np.arange(my, max(yal)+1,fov))
                        # if np.mod((min(yal)-1),fov)!=0:
                        #     ally=[math.floor(min(yal)/fov)*fov ]+ally
                        # else:
                        #     ally=[min(yal)]+ally
                        if len(ally)==0:
                            ally=[min(yal)]
                    # if ally[-1]<max(yal)-fov:
                    #     ally=ally+[max(yal)-fov]
                    # if fov==13:
                    #     print(f"why y {fov} {(min(yal), max(yal)) }  {np.array(ally)}")
                    #     if 0 in ally:
                    #         print(f"seeeem \n")
                    for y in ally:
                        self.WPCHeight[tak,wpc[0]].add((x+fov/2,y+fov/2,wpc[0]))    
                        #self.WPCs.append((x+fov/2,y+fov/2,wpc[0]))
                        conn=(self.Distance((x+fov/2,y+fov/2,wpc[0]), self.GCloc)<=self.drone.range)
                        self.Connection[x+fov/2,y+fov/2,wpc[0]]=conn
                        ################################### NO Connection! 
                        if conn==True:
                            self.Con_wpc.append((x+fov/2,y+fov/2,wpc[0]))
                            self.ShortUpload[(x+fov/2,y+fov/2,wpc[0])]=(self.drone.loiter,(x+fov/2,y+fov/2,wpc[0])) # if is connected, it is itself. 
                        else:
                            Dis=[self.Distance((x+fov/2,y+fov/2,wpc[0]), ww) for ww in UW]
                            wm=Dis.index(min(Dis))
                            Ttime=min(Dis)/self.drone.speed+self.drone.loiter # maybe 1 s
                            self.ShortUpload[(x+fov/2,y+fov/2,wpc[0])]=(Ttime,UW[wm])
                            self.WPCs.append(UW[wm])
                            self.Connection[UW[wm]]=True
                            self.ShortUpload[UW[wm]]=(self.drone.loiter,UW[wm])
                        ########################### For get the coverage area
                        #if self.Cover_map.get((x+fov/2,y+fov/2,wpc[0]))==None:
                        cx=int(x)
                        while cx<x+fov:
                           cy=int(y) 
                           while cy<y+fov:
                                if self.Cover_map.get((x+fov/2,y+fov/2,wpc[0]))==None:
                                    self.Cover_map[x+fov/2,y+fov/2,wpc[0]].append((cx,cy,tak))
                                    self.Area_WPC[(cx,cy),tak].append((x+fov/2,y+fov/2,wpc[0]))
                                elif (cx,cy,tak) not in self.Cover_map[x+fov/2,y+fov/2,wpc[0]]:
                                    self.Cover_map[x+fov/2,y+fov/2,wpc[0]].append((cx,cy,tak))
                                    self.Area_WPC[(cx,cy),tak].append((x+fov/2,y+fov/2,wpc[0]))
                                cy=cy+1
                           cx=cx+1
                        ########################
        self.SetCoveWPC()
    
    
    def initialTask(self):     #Here we only consider to handle started task!
        for a in self.Area_Task:
            for t in self.Area_Task[a]: 
                # Here t is (task, (rt, et)) 
                offset= (max(t[1][0],self.time)-self.time)%((self.MissionDict[t[0]][0]))*60
                offset=0 # Do not split on offset
                #Here we have release time and ending time!!!!! 
                self.TaskState[a][t[0],offset]=(0,0)# Deadline job reward, stored reward. 
                
                self.Area_Mission_time[a][t[0],offset]=(t[1][0]*60, t[1][1]*60 if t[1][1]!=-1 else -1)
        ########### Here initialTask have problem, 
        #default is 0,0 but it is not! 
    def ClusterTask(self):# cluster tasks based on their mission and offset
        tmp_jr=defaultdict(list)
        tmp_sr=defaultdict(list)
        for a in self.TaskState:
            #print(f"see area {a}")
            for m,st in self.TaskState[a].items():
                mission,offset=m
                jr,sr=st
                self.Mission_Area[mission,offset].append(a)
                ############# Add grid issue
                grid=self.Area_grid[a]
                #print(f"check release time {self.Area_Task[a]}")
                self.Mission_Grid[mission,offset].add((mission,offset,grid))
                self.Grid_Area[mission,offset,grid].append(a)
                rt, et=self.Area_Mission_time[a][mission,offset]
                # if mission=='FT':
                #     print(f"see area {a} {rt} {et}")
                if (mission,offset) not in list(tmp_jr.keys()):
                    tmp_jr[mission,offset]=[]
                if (mission,offset) not in list(tmp_sr.keys()):
                    tmp_sr[mission,offset]=[]
                if rt<self.time+((self.MissionDict[mission][0]))*60 : ############################# Add a release time judgement! 
                    self.Grid_toCov[mission,offset,grid].append(a) # To be update at StateTrans 
                    self.Fix_toCov[mission,offset,grid].append(a)
                    tmp_jr[mission,offset].append((jr,a))
                    if sr>0:
                        tmp_sr[mission,offset].append((sr,a))
        for key in tmp_jr:
            dict_jr=defaultdict(list)
            for p in tmp_jr[key]:
                dict_jr[p[0]].append(p[1])
            self.Mission_JR[key]=dict_jr
            dict_sr=defaultdict(list)
            for p in tmp_sr[key]:
                dict_sr[p[0]].append(p[1])                
            self.Mission_SR[key]=dict_sr
            m, offset=key
            self.Mission_DL[key]=offset+((self.MissionDict[key[0]][0])*60)
            if IFLOG:
                self.logfile.write(f"Add {self.drone.id} {self.time} {key[0]} {len(tmp_jr[key])}\n")
            self.logtasks[key[0]][self.time]=len(tmp_jr[key]) # Generated tasks
    def CheckReleasetime(self,a, mm, dl, time, period):
        ############## dl: Current deadline)no need to be current dl), time: current time greater than dl
        rt, et=self.Area_Mission_time[a][mm]
        k=np.floor((time-dl)/(period*60))
        if np.mod(time-dl,period*60)==0:
            k=k-1
        dltime=dl+(k+1)*(period*60)
        # if rt<=time and (et>=dltime or et==-1):
        #     print(f"1? ")
        #     return True
        if rt<dltime and (et>=dltime or et==-1):
            return True
        # else:
        #     print(f"Norelease {rt} {et} {time} {dltime} {period} {mm} ")
        return False
        
    def ClusterStateTrans(self,wpc):  
        Reward=0; SReward=0; Penalty=0
        NoUpDL=True
        if self.wp!=wpc:
            distance=self.DisMatrix[self.wp][wpc]
        else:
            distance=0
        Ttime=distance/self.drone.speed+self.drone.loiter # maybe 1 s
        self.time=self.time+Ttime
        # if IFLOG:
        #     logfile.write(f"Drone {self.drone.id} at {self.time}\n")
        self.wp=wpc
        CONNECT=self.Connection[wpc]
        areas=self.Cover_map.get(wpc)
        ############ Update time 
        for m , dl in self.Mission_DL.items():  # Check the dl! 
            # if IFLOG:
            #     logfile.write(f"Drone {self.drone.id} JR  {m} {len(self.Mission_JR[m][0])}\n")
            if dl<self.time:
                # if IFLOG:
                #     logfile.write(f"Drone {self.drone.id} Miss Deadline  {m} DL {dl} at {self.time}\n")
                                ###############################
                NoUpDL=False
                period=self.MissionDict[m[0]][0]
                for grid in self.Mission_Grid[m]:
                    #mm, offset, gid =grid
                    CurArea=[a for a in self.Grid_Area[grid] if self.CheckReleasetime(a,grid[:-1], dl, self.time, period)]
                    self.Grid_toCov[grid]=CurArea  # Update to Cover
                    self.Fix_toCov[grid]=copy.copy(CurArea)
                    if IFLOG:
                        self.logfile.write(f"Add {self.drone.id} {self.time} {dl} {m} {len(CurArea)}\n")
                    self.logtasks[m[0]][dl]=len(CurArea)
                    #print(f"Add {self.drone.id} {self.time} {m} {len(CurArea)}\n")
                weight=1
                #k=np.floor((self.time-dl)/(period*60))
                period=self.MissionDict[m[0]][0]
                k=np.floor((self.time-dl)/(period*60))
                if np.mod(self.time-dl,period*60)==0:
                    k=k-1
                ndl=dl+(k+1)*(period*60)
                self.Mission_DL[m]=ndl  # 
                NumMiss=0
                #print(f"k {k} ")
                if k>0:
                    for i in range(int(k)): # 0 to k-1 => 1 to k
                        Cur_Mission=[a for a in self.Mission_Area.get(m) if self.CheckReleasetime(a,m, dl, dl+period*(i+1), period)]
                        NumMiss=NumMiss+len(Cur_Mission)
                Cur_Mission=[a for a in self.Mission_Area.get(m) if self.CheckReleasetime(a,m, dl, self.time, period)]
                ######### Get tasks released in last period.
                Cur_miss=[a for a in self.Mission_JR[m][0] if self.CheckReleasetime(a,m, dl-period*60, dl, period)]
                jrcount=len(Cur_miss)
                if IFLOG:
                    self.logfile.write(f"Miss {self.drone.id} {self.time} {m} {dl} {jrcount}\n")
                self.logmiss[m[0]][dl]=jrcount # log miss
                #print(f"Miss {m}  {jrcount} weight {weight}  at {self.time}\n")
                Penalty=Penalty+(k*weight*self.sigma)*NumMiss+(jrcount*(weight*self.sigma))
                Reward=Reward-(k*weight*self.sigma)*NumMiss-(jrcount*(weight*self.sigma))   # Get the penalty! 
                for a in Cur_Mission:
                    self.TaskState[a][m]=(0,0) # update the jr and sr       
                self.Mission_JR[m]=defaultdict(list) 
                self.Mission_JR[m][0]=list(Cur_Mission)  # = =??? have to add list??s cluster update all is 0
                self.Mission_SR[m]=defaultdict(list)      # cluster update for sr=0
        ################## Upload Storage
        if CONNECT:
            for m , srdict in self.Mission_SR.items():  # clear the storage!!! 
                mission,offset=m
                for sv, ars in srdict.items():
                    for a in ars:
                        jr, sr=self.TaskState[a][m]
                        if sr>jr:
                            try:
                                self.Mission_JR[m][jr].remove(a)  
                                self.Mission_JR[m][sr].append(a)  
                                Reward=round(Reward+(sr-jr),3)  # Get the reward from storage! 
                            except:
                                pass
                                # print(f"see reward {Reward}")
                        self.TaskState[a][m]=(max(sr,jr),0) # update sr=0
                self.Mission_SR[m]=defaultdict(list)   # clean storage! 
        if areas!=None: 
            for a in areas:   # the areas the waypoint can cover!!!! 
                a=a[:-1]
                for m,st in self.TaskState[a].items():
                    mission,offset=m
                    jr,sr=st
                    ############ Get the reward!!!! 
                    poheights=list(self.drone.WPCinfo.get(mission).keys())
                    #poheights=list(self.WPC_TakFov.get(mission).keys())
                    z=wpc[2]
                    if z>max(poheights):
                        rwd=0
                    else:
                        z=min([k for k in poheights if k>=z])
                        rwd=self.drone.WPCinfo.get(mission).get(z)[1] 
                    #######################################   
                    if jr==0 and sr==0 and rwd>0: # Which means it is covered! 
                        grid=self.Area_grid.get(a) # this area his this mission, area in this grid, the grid 
                        #print(f"why {a} {m,grid} {self.Grid_toCov[m[0],m[1],grid]} ")
                        try:
                            self.Grid_toCov[m[0],m[1],grid].remove(a)
                            #print(f"GridtoCov {m[0],m[1],grid} remove {a}")
                        except:
                            pass
                    ########################################
                    if CONNECT and rwd>jr:
                        self.TaskState[a][m]=(rwd,0) # update sr=0
                        try:
                            self.Mission_JR[m][jr].remove(a)  
                            self.Mission_JR[m][rwd].append(a)  
                            Reward=round(Reward+(rwd-jr),3) # Add reward!!! 
                            self.TaskState[a][m]=(rwd,0) # update sr=0
                        except:
                            pass
                            #print(f"maybe area is not released {a}")
                    elif not CONNECT and rwd>max(sr,jr):   # if SR<JR is useless, right???  
                        try:
                            if sr>0:
                                self.Mission_SR[m][sr].remove(a)  # update state
                            self.Mission_SR[m][rwd].append(a)   # update cluster state 
                            #print(f"see why round {SReward}")
                            SReward=round(SReward+(rwd-max(sr,jr)),3)
                            self.TaskState[a][m]=(jr,rwd) # update sr=0
                            #print(f"see why round {SReward} {rwd-max(sr,jr)} {rwd} {sr} {jr}")
                        except:
                            pass
                            #print(f"maybe area is not released {a}")
        if IFLOG:
            if Reward==0 and SReward==0 and Penalty==0:
                self.logfile.write(f"Hover {self.drone.id} {self.time}\n")
            else:
                self.logfile.write(f"Reward {self.drone.id} {self.time} {Reward} {SReward} {Penalty}\n")
                self.logReward[self.time]=(Reward,SReward,Penalty)

        #print(f"see reward {Reward},sreward {SReward}, penalty  time {Ttime} {self.time}")
        return Reward, SReward, Penalty, NoUpDL

    def GetDataReward(self,wpc): # Get reward without updating state!!! 
        Reward=0; Covercount=0
        if self.wp!=wpc:
            distance=self.DisMatrix[self.wp][wpc]
        else:
            distance=0
        Ttime=distance/self.drone.speed+self.drone.loiter # maybe 1 s
        areas=self.Cover_map.get(wpc)
        if areas==None:
            return 0,self.drone.loiter,0
        ############ Update time 
        for a in areas:   # the areas the waypoint can cover!!!! 
            #print(f"see area {a[:-1]}")
            for m,st in self.TaskState[(a[:-1])].items():
                #print(m,st)
                grid=list(self.Mission_Grid[m])[0]
                if a[:-1] in self.Fix_toCov[grid]:
                    #print(f"see {m} {st}")
                    mission,offset=m
                    if self.Mission_DL[m]<self.time+Ttime:
                        jr,sr=(0,0)
                    else:
                        jr,sr=st
                    ############ Get the reward!!!! 
                    poheights=list(self.drone.WPCinfo.get(mission).keys())
                    z=wpc[2]
                    if z>max(poheights):
                        rwd=0
                    else:
                        z=min([k for k in poheights if k>=z])
                        rwd=self.drone.WPCinfo.get(mission).get(z)[1] 
                    #######################################    
                    if rwd>max(sr,jr):   # if SR<JR is useless, right???  
                        Reward=Reward+(rwd-max(sr,jr))
                        if sr==0 and jr==0 and self.Mission_DL[m]>self.time+Ttime:
                            Covercount=Covercount+1
        #print(f"see expected {Reward}")# penalty {Penalty}")
        return Reward, Ttime, Covercount
    
    def GetFixDataReward(self,wpc): # Get reward without updating state!!! 
        Reward=0; Covercount=0
        # if len(cwp)==0:
        #     cwp=self.wp
        if self.wp!=wpc:
            distance=self.DisMatrix[self.wp][wpc]
        else:
            distance=0
        Ttime=distance/self.drone.speed+self.drone.loiter # maybe 1 s
        areas=self.Cover_map.get(wpc)
        if areas==None:
            return 0,self.drone.loiter,0
        ############ Update time 
        for a in areas:   # the areas the waypoint can cover!!!! 
            #print(f"see area {a[:-1]}")
            for m,st in self.TaskState[(a[:-1])].items():
                #print(m,st)
                grid=list(self.Mission_Grid[m])[0]
                if a[:-1] in self.Fix_toCov[grid]:
                    #print(f"see {m} {st}")
                    mission,offset=m
                    if self.Mission_DL[m]<self.time+Ttime:
                        return Reward, Ttime, Covercount
                        #jr,sr=(0,0)
                    else:
                        jr,sr=st
                    ############ Get the reward!!!! 
                    poheights=list(self.drone.WPCinfo.get(mission).keys())
                    z=wpc[2]
                    if z>max(poheights):
                        rwd=0
                    else:
                        z=min([k for k in poheights if k>=z])
                        rwd=self.drone.WPCinfo.get(mission).get(z)[1] 
                    #######################################    
                    if rwd>max(sr,jr):   # if SR<JR is useless, right???  
                        Reward=Reward+(rwd-max(sr,jr))
                        if sr==0 and jr==0:
                            Covercount=Covercount+1
        #print(f"see expected {Reward}")# penalty {Penalty}")
        return Reward, Ttime, Covercount

    
    def Distance(self,w1, w2): #Here z did not multiple Res
        dis=np.sqrt(((w1[0]-w2[0])*self.Res)**2+((w1[1]-w2[1])*self.Res)**2+((w1[2]-w2[2]))**2)
        return dis
    
    def DrawWPsequence(self):
        fig=plt.figure()
        ax=plt.axes(projection='3d')
        ax.plot3D([i[0] for i in self.waypointSeq], [i[1] for i in self.waypointSeq],[i[2] for i in self.waypointSeq])
        ax.scatter([i[0] for i in self.waypointSeq], [i[1] for i in self.waypointSeq],[i[2] for i in self.waypointSeq])
        conw=[w for w in self.waypointSeq ]
        plt.show()
    
    
    def NeigWPC(self,areas,g): ### Now it is stupid! 1) Add checking reward method! 2) How to handle staying at the same locations? 
        mission, offset,gid=g
        WPCs=[]
        poheights=list(self.drone.WPCinfo.get(g[0]).keys())
        for a in areas: 
            to_wp=[w for w in self.Area_WPC.get(a) if w[2] in (poheights)]
            maxh=max([ww[2] for ww in to_wp])
            to_wp=[w for w in to_wp if w[2]==maxh]
            WPCs=WPCs+to_wp
        WPCs=list(set(WPCs))
        WPCs=[w for w in WPCs if not (self.CheckReturn(w,((g[0],g[1]),self.Mission_DL[g[0],g[1]]))) ]
        if len(WPCs)==0:
            nwpc=self.ShortUpload.get(self.wp)[1]
            #print(f"No WPCs is valid????")
        else:
            Diss=[self.Distance(self.wp,w) for w in WPCs] #Here the distance can be compute less????
            adex=Diss.index(min(Diss))
            nwpc=WPCs[adex]
        return nwpc
    
    def BestCovNeigWPC(self,areas,g): ### Now it is stupid! 1) Add checking reward method! 2) How to handle staying at the same locations? 
        mission, offset,gid=g
        WPCs=[]
        poheights=[h[0] for h in  self.WPC_TakFov[g[0]]]
        for a in areas: 
            try:
                #to_wp=[w for w in self.Area_WPC.get((a,mission)) if w[2]==FHeight]
                to_wp=[w for w in self.Area_WPC.get((a,mission)) if w[2] in ((poheights))]
                to_wp=[w for w in to_wp if w in self.WPCs]
                WPCs=WPCs+to_wp
            except:
                pass
        WPCs=list(set(WPCs))
        #print([self.CheckReturn(w,((g[0],g[1]),self.Mission_DL[g[0],g[1]])) for w in WPCs])
        WPCs=[w for w in WPCs if not (self.CheckReturn(w,((g[0],g[1]),self.Mission_DL[g[0],g[1]]))) ]
        if len(WPCs)==0:
            #nwpc=self.ShortUpload.get(self.wp)[1]
            #print(f"No WPCs is valid--> Return back!")
            return [], True
        else:
            Covlist=[self.GetDataReward(w) for w in WPCs]
            maxrwd=max([x[2]/x[1] for x in Covlist])
            tmp=[i for i in list(range(len(Covlist))) if Covlist[i][2]/Covlist[i][1]==maxrwd]
            x,y,z=max(Covlist, key=lambda x: x[2]/x[1])
            #Diss=[self.Distance(self.wp,w) for w in WPCs] #Here the distance can be compute less????
            adex=Covlist.index((x,y,z))
            nwpc=WPCs[adex]
            return nwpc, False

    def BestTopDownNeigWPC(self,areas,g,SugestH=0): ### Now it is stupid! 1) Add checking reward method! 2) How to handle staying at the same locations? 
        mission, offset,gid=g
        WPCs=[]
        poheights=[h[0] for h in  self.WPC_TakFov[g[0]]]
        #print(f"see height! {mission}   {self.WPC_TakFov[g[0]]}")
        if SugestH==0:
            FHeight=max(poheights)
        else:
            FHeight=SugestH
        for a in areas: 
            
            try:
                to_wp=[w for w in self.Area_WPC.get((a,mission)) if w[2]==FHeight]
                #to_wp=[w for w in self.Area_WPC.get((a,mission)) if w[2] in ((poheights))]
                to_wp=[w for w in to_wp if w in self.WPCs]
                WPCs=WPCs+to_wp
            except:
                pass
        WPCs=list(set(WPCs))
        WPCs=[w for w in WPCs if not (self.CheckReturn(w,((g[0],g[1]),self.Mission_DL[g[0],g[1]]))) ]
        #WPCs=[w for w in WPCs if not (self.CheckReturn(w,[])) ]
        if len(WPCs)==0:
            #print(f"No WPCs is valid--> Return back!")
            return [], True
        else:
            Covlist=[(self.GetDataReward(w),w) for w in WPCs]
            Covlist=[x for x in Covlist if x[0][2]>0]
            CoWPCs=[x[1] for x in Covlist]
            mint=min(x[0][1] for x in Covlist)
            # if IFLOG:
            #     logfile.write(f"Covlist {mint} {[WPCs[i] for i in tmp]}\n")
            tmp=[i for i in list(range(len(Covlist))) if Covlist[i][0][1]==mint]
            poWP=[CoWPCs[i] for i in tmp]
            if len(poWP)>1:
                poWP.sort(key=lambda s: (s[1]-self.wp[1])**2) # Get the earliest deadline. 
                #poWP.sort(key=lambda s: (s[0]-self.wp[0])**2) # Get the earliest deadline. 
                #print(poWP)
            nwpc=poWP[0]
            return nwpc, False
    ############################################################# Ajust NN
    def Flytime(self,wp1, wp2):
        if wp1!=wp2:
            distance=self.DisMatrix[wp1][wp2]
        else:
            distance=0
        Ttime=distance/self.drone.speed+self.drone.loiter # maybe 1 s
        return Ttime
    
    def QuickTopDown(self,areas,wpcs, ctime,DL):
        cw=self.wp
        WPC=copy.copy(wpcs)
        #Area=copy.copy(areas)
        cutime=ctime
        WSeq=[]
        CovA=set()           
        UpTime=min(list(self.Mission_DL.values()))   
        while len(set(areas)-CovA)>0 : # Nearest Neighbor 
            #print([[a[:-1] for a in self.Cover_map[w]] for w in wps])
            ft=[(self.Flytime(cw, w),w) for w in WPC] 
            mint=min([t[0] for t in ft])    
            poWP=[ww[1] for ww in ft  if ww[0]==mint]
            if len(poWP)>1:
                poWP.sort(key=lambda s: (s[1]-cw[1])**2) # Get the earliest deadline. 
            dw=poWP[0]
            cutime=cutime+self.Flytime(cw, dw)
            cw=dw   
            cov=set([a[:-1] for a in self.Cover_map[dw]]).intersection(set(areas)) 
            CovA.update(cov)
            WSeq.append(cw)
            WPC.remove(dw)
            WPC=[w for w in WPC if not self.QuickCheck(cutime, DL, cw, w)]
            WPC=[w for w in WPC if len(set([a[:-1] for a in self.Cover_map[w]]).intersection(set(areas))-CovA)>0]
            if len(WPC)==0 or cutime>UpTime:
            #if len(WPC)==0:
                #print(f"Out of Time?")
                break
        #print(f" Nowpc? Height {self.wp[-1]} {len(WPC)} {ctime} {DL} {cutime-ctime} {WSeq} {len(CovA)} {len(set(areas)-CovA)}")
        return len(CovA), cutime-ctime, WSeq
    
    def QuickCoverage(self,areas,AllWPCs,ctime,DL): # For one 
        cw=self.wp
        cutime=ctime
        WSeq=[]
        WPC=copy.copy(AllWPCs)
        CovA=set() 
        UpTime=min(list(self.Mission_DL.values()))   
        #WPCs=[w for w in WPCs if not self.QuickCheck(cutime, DL, cw, w)]
        while len(set(areas)-CovA)>0 : # 
            if len(WPC)==0 or cutime>UpTime:
                break
            Covlist=[(set([a[:-1] for a in self.Cover_map[w]]).intersection(set(areas)-CovA), self.Flytime(cw,w)) for w in WPC]
            x,y=max(Covlist, key=lambda x: (len(x[0]))/(x[1]))
            #print(f"\n cover {x} {y} ")
            adex=Covlist.index((x,y))
            dw=WPC[adex]
            cutime=cutime+self.Flytime(cw, dw)
            cw=dw  
            CovA.update(x)
            WSeq.append(cw)
            WPC.remove(dw)
            WPC=[w for w in WPC if not self.QuickCheck(cutime, DL, cw, w)]
            WPC=[w for w in WPC if len(set([a[:-1] for a in self.Cover_map[w]]).intersection(set(areas)-CovA))>0]
            
        return len(CovA), cutime-ctime, WSeq
    
######################################################################
        #return trat, WSeq
    def QuickCheck(self,ctime,DL,cwp, wp ):
        Delay=self.Flytime(cwp, wp)
        upt, _=self.ShortUpload.get(wp)
        Delay=Delay+upt
        if DL-ctime<Delay:
            return True
        else:
            return False
    def AdjustHeightNeigWPC(self,areas,g, FPnum=0): ### Now it is stupid! 1) Add checking reward method! 2) How to handle staying at the same locations? 
        mission, offset,gid=g
        poheights=[h[0] for h in  self.WPC_TakFov[g[0]]]
        ##########Get current DL
        DL=self.Mission_DL[g[:2]]
        stdl=[]
        if len(self.Mission_SR)>0:
            stdl=[self.Mission_DL[x[0]] for x in list(self.Mission_SR.items()) if len(x[1])>0]
        #if len(stdl)>0: 
        #min(list(self.Mission_DL.values()))
        DL=min(stdl+[DL]+[self.EndTime])
        Comh=[]
        ##################
        AllWPCs=[]
        for h in poheights:
            WPCs=[]
            for a in areas: 
                try:
                    to_wp=[w for w in self.Area_WPC.get((a,mission)) if w[2]==h]
                    to_wp=[w for w in to_wp if w in self.WPCs]
                except:
                    pass
                WPCs=WPCs+to_wp
            WPCs=list(set(WPCs))
            WPCs=[w for w in WPCs if not (self.CheckReturn(w,((g[0],g[1]),self.Mission_DL[g[0],g[1]]))) ]
            AllWPCs=AllWPCs+WPCs
            #print(f"Mission {(mission,offset)} {h}")
            if len(WPCs)>0:
                Covnum, Ctime, WSeq=self.QuickTopDown(areas,WPCs, self.time,DL)
                Comh.append([h,Covnum, Ctime, WSeq])
        if FPnum==6:
            Covnum, Ctime, WSeq=self.QuickCoverage(areas, AllWPCs, self.time, DL)
            if len(WSeq)>0:
                Comh.append((0,Covnum, Ctime, WSeq))
        # print(f"seee {[i[:3] for i in Comh]}")
        # print(f"see Cover {(Covnum, Ctime, WSeq)} ")
        if len(Comh)==0: # Nowpc
            return 0, 0, 0, [],True##### Compare: 1) CovNum--> Ctime 
        MaxCnum=max(i[1] for i in Comh)
        Items=[i for i in Comh if i[1]==MaxCnum]
        Suh,Cn,Ctime,wseq=sorted(Items,key=lambda x:x[2])[0]
        #if Suh==0:
            #print(f"Use Coverage path! {Comh} ")
        return  Suh,Cn,Ctime,wseq, False
    ###########################################################################   
    
    def DistanceNeigWPC(self,areas,g): ### Now it is stupid! 1) Add checking reward method! 2) How to handle staying at the same locations? 
        mission, offset,gid=g
        WPCs=[]
        #poheights=list(self.drone.WPCinfo.get(g[0]).keys())
        poheights=[h[0] for h in  self.WPC_TakFov[g[0]]]
        for a in areas: 
            to_wp=[w for w in self.Area_WPC.get((a,mission)) if w[2] in (poheights)]
            to_wp=[w for w in to_wp if w in self.WPCs]
            WPCs=WPCs+to_wp
        WPCs=list(set(WPCs))
        WPCs=[w for w in WPCs if not (self.CheckReturn(w,((g[0],g[1]),self.Mission_DL[g[0],g[1]]))) ]
        if len(WPCs)==0:
            #print(f"No WPCs is valid--> Return back!")
            return [], True
        else:
            Covlist=[(self.GetDataReward(w),w) for w in WPCs]
            Covlist=[x for x in Covlist if x[0][2]>0]
            CoWPCs=[x[1] for x in Covlist]
            mint=min(x[0][1] for x in Covlist)
            tmp=[i for i in list(range(len(Covlist))) if Covlist[i][0][1]==mint]
            poWP=[CoWPCs[i] for i in tmp]
            nwpc=poWP[0]
            return nwpc, False
        
    def RewardNeigWPC(self,areas,g): ### Now it is stupid! 1) Add checking reward method! 2) How to handle staying at the same locations? 
        mission, offset,gid=g
        WPCs=[]
        poheights=[h[0] for h in  self.WPC_TakFov[g[0]]] #speed up it
        for a in areas: 
            to_wp=[w for w in self.Area_WPC.get((a,mission)) if w[2] in (poheights)]
            to_wp=[w for w in to_wp if w in self.WPCs]
            # maxh=max([ww[2] for ww in to_wp])
            # to_wp=[w for w in to_wp if w[2]==maxh]
            WPCs=WPCs+to_wp
        WPCs=list(set(WPCs))
        WPCs=[w for w in WPCs if not (self.CheckReturn(w,((g[0],g[1]),self.Mission_DL[g[0],g[1]]))) ]
        if len(WPCs)==0:
            nwpc=self.ShortUpload.get(self.wp)[1]
            #print(f"No WPCs is valid--> Return back!")
            return nwpc, True
        else:
            Covlist=[self.GetFixDataReward(w) for w in WPCs]   ###### Here change this!!!
            x,y,z=max(Covlist, key=lambda x: (x[0])/(x[1]))
            #print(f"check {x} {y} {z} ")
            if x==0:
                nwpc=self.ShortUpload.get(self.wp)[1]
                return nwpc,True
            #print(f"which cover more? {(x,y,z)}")
            #Diss=[self.Distance(self.wp,w) for w in WPCs] #Here the distance can be compute less????
            adex=Covlist.index((x,y,z))
            nwpc=WPCs[adex]
            return nwpc, False
    
    def RewardWPC(self,areas,g): ### Now it is stupid! 1) Add checking reward method! 2) How to handle staying at the same locations? 
        mission, offset,gid=g
        WPCs=[]
        poheights=[h[0] for h in  self.WPC_TakFov[g[0]]] #speed up it
        for a in areas: 
            to_wp=[w for w in self.Area_WPC.get((a,mission)) if w[2] in (poheights)]
            to_wp=[w for w in to_wp if w in self.WPCs]
            # maxh=max([ww[2] for ww in to_wp])
            # to_wp=[w for w in to_wp if w[2]==maxh]
            WPCs=WPCs+to_wp
        WPCs=list(set(WPCs))
        WPCs=[w for w in WPCs if not (self.CheckReturn(w,((g[0],g[1]),self.Mission_DL[g[0],g[1]]))) ]
        if len(WPCs)==0:
            nwpc=self.ShortUpload.get(self.wp)[1]
            #print(f"No WPCs is valid--> Return back!")
            return nwpc, True
        else:
            Covlist=[self.GetFixDataReward(w) for w in WPCs]   ###### Here change this!!!
            x,y,z=max(Covlist, key=lambda x: (x[0])/(x[1]))
            #print(f"check {x} {y} {z} ")
            if x==0:
                nwpc=self.ShortUpload.get(self.wp)[1]
                return nwpc,True
            #print(f"which cover more? {(x,y,z)}")
            #Diss=[self.Distance(self.wp,w) for w in WPCs] #Here the distance can be compute less????
            adex=Covlist.index((x,y,z))
            nwpc=WPCs[adex]
            return nwpc, False
    
    
    
    
    
    
    def CoverNeigWPC(self,areas,g): ### Now it is stupid! 1) Add checking reward method! 2) How to handle staying at the same locations? 
        mission, offset,gid=g
        WPCs=[]
        poheights=[h[0] for h in  self.WPC_TakFov[g[0]]] #speed up it
        for a in areas: 
            to_wp=[w for w in self.Area_WPC.get((a,mission)) if w[2] in (poheights)]
            to_wp=[w for w in to_wp if w in self.WPCs]
            # maxh=max([ww[2] for ww in to_wp])
            # to_wp=[w for w in to_wp if w[2]==maxh]
            WPCs=WPCs+to_wp
        WPCs=list(set(WPCs))
        WPCs=[w for w in WPCs if not (self.CheckReturn(w,((g[0],g[1]),self.Mission_DL[g[0],g[1]]))) ]
        if len(WPCs)==0:
            nwpc=self.ShortUpload.get(self.wp)[1]
            #print(f"No WPCs is valid--> Return back!")
            return nwpc, True
        else:
            Covlist=[self.GetFixDataReward(w) for w in WPCs]   ###### Here change this!!!
            x,y,z=max(Covlist, key=lambda x: (x[2])/(x[1]))
            #print(f"\n cover {x} {y} {z} ")
            if z==0:
                nwpc=self.ShortUpload.get(self.wp)[1]
                return nwpc,True
            #print(f"which cover more? {(x,y,z)}")
            #Diss=[self.Distance(self.wp,w) for w in WPCs] #Here the distance can be compute less????
            adex=Covlist.index((x,y,z))
            nwpc=WPCs[adex]
            return nwpc, False
        
#######################################################    
    
    def MultipleCovWPC(self,CurGrids): 
        Areas=[]
        WPCs=[]
        for g in CurGrids:
            areas=self.Grid_toCov[g]
            mission, offset,gid=g
            poheights=[h[0] for h in  self.WPC_TakFov[g[0]]] #speed up it
            for a in areas: 
                to_wp=[w for w in self.Area_WPC.get((a,mission)) if w[2] in (poheights)]
                to_wp=[w for w in to_wp if w in self.WPCs]
                # maxh=max([ww[2] for ww in to_wp])
                # to_wp=[w for w in to_wp if w[2]==maxh]
                WPCs=WPCs+to_wp
        WPCs=list(set(WPCs))
        WPCs=[w for w in WPCs if not (self.CheckReturn(w,((g[0],g[1]),self.Mission_DL[g[0],g[1]]))) ]
        if len(WPCs)==0:
            nwpc=self.ShortUpload.get(self.wp)[1]
            #print(f"No WPCs is valid--> Return back!")
            return nwpc, True
        else:
            Covlist=[self.GetFixDataReward(w) for w in WPCs]   ###### Here change this!!!
            x,y,z=max(Covlist, key=lambda x: (x[2])/(x[1]))
            #print(f"\n cover {x} {y} {z} ")
            if z==0:
                nwpc=self.ShortUpload.get(self.wp)[1]
                return nwpc,True
            #print(f"which cover more? {(x,y,z)}")
            #Diss=[self.Distance(self.wp,w) for w in WPCs] #Here the distance can be compute less????
            adex=Covlist.index((x,y,z))
            nwpc=WPCs[adex]
            return nwpc, False
        
    def MultipleRewardWPC(self,CurGrids): 
        Areas=[]
        WPCs=[]
        for g in CurGrids:
            areas=self.Fix_toCov[g]
            mission, offset,gid=g
            poheights=[h[0] for h in  self.WPC_TakFov[g[0]]] #speed up it
            for a in areas: 
                to_wp=[w for w in self.Area_WPC.get((a,mission)) if w[2] in (poheights)]
                to_wp=[w for w in to_wp if w in self.WPCs]
                # maxh=max([ww[2] for ww in to_wp])
                # to_wp=[w for w in to_wp if w[2]==maxh]
                WPCs=WPCs+to_wp
        WPCs=list(set(WPCs))
        #print(f"WPC? {WPCs}")
        WPCs=[w for w in WPCs if not (self.CheckReturn(w,((g[0],g[1]),self.Mission_DL[g[0],g[1]]))) ]
        if len(WPCs)==0:
            nwpc=self.ShortUpload.get(self.wp)[1]
            #print(f"No WPCs is valid--> Return back!")
            return nwpc, True
        else:
            Covlist=[self.GetFixDataReward(w) for w in WPCs]   ###### Here change this!!!
            x,y,z=max(Covlist, key=lambda x: (x[0])/(x[1]))
            #print(f"\n Reward cover {x} {y} {z} ")
            if x==0:
                #nwpc=self.ShortUpload.get(self.wp)[1]
                return [],True
            #print(f"which cover more? {(x,y,z)}")
            #Diss=[self.Distance(self.wp,w) for w in WPCs] #Here the distance can be compute less????
            adex=Covlist.index((x,y,z))
            nwpc=WPCs[adex]
            return nwpc, False
###################################################

    def ImproveRewardWPC(self,areas,g): ### Now it is stupid! 1) Add checking reward method! 2) How to handle staying at the same locations? 
        #print(f" Enter here")
        mission, offset,gid=g
        WPCs=[]
        #poheights=list(self.drone.WPCinfo.get(g[0]).keys())
        poheights=[h[0] for h in  self.WPC_TakFov[g[0]]] #speed up it
        #poheights.remove(max(poheights))
        #print(f"why", poheights)
        if len(poheights)==0:
            nwpc=self.ShortUpload.get(self.wp)[1]
            #print(f"No WPCs is valid--> Return back!")
            return nwpc, True
        for a in areas: 
            to_wp=[w for w in self.Area_WPC.get((a,mission)) if w[2] in (poheights)]
            to_wp=[w for w in to_wp if w in self.WPCs]
            # maxh=max([ww[2] for ww in to_wp])
            # to_wp=[w for w in to_wp if w[2]==maxh]
            WPCs=WPCs+to_wp
        WPCs=list(set(WPCs))
        #WPCs=[w for w in WPCs if not (self.RelexCheckReturn(w,((g[0],g[1]),self.Mission_DL[g[0],g[1]]))) ]
        WPCs=[w for w in WPCs if not (self.CheckReturn(w,[])) ]

        if len(WPCs)==0:
            nwpc=self.ShortUpload.get(self.wp)[1]
            #print(f"No WPCs is valid--> Return back!")
            return nwpc, True
        else:
            Covlist=[self.GetDataReward(w) for w in WPCs]
            x,y,z=max(Covlist, key=lambda x: x[0]/x[1])
            #print(f"See {x} {y} {z}")
            if x==0:
                #print(f"get here??")
                nwpc=self.ShortUpload.get(self.wp)[1]
                #print(f"No Improve!!")
                return nwpc,True
            #coverage_first! 
            x,y,z=max(Covlist, key=lambda x: x[2]/x[1])
            #print(f"See 2 {x} {y} {z}")
            if z!=0:
                #print(f"No Cover")
                adex=Covlist.index((x,y,z))
                nwpc=WPCs[adex]
            else:
                x,y,z=max(Covlist, key=lambda x: x[0]/x[1])
                adex=Covlist.index((x,y,z))
                nwpc=WPCs[adex]
            # x,y,z=max(Covlist, key=lambda x: x[0]/x[1])
            # adex=Covlist.index((x,y,z))
            # nwpc=WPCs[adex]
            #print(f"which cover more? {(x,y,z)}")
            #Diss=[self.Distance(self.wp,w) for w in WPCs] #Here the distance can be compute less????
            #print(f"See {x} {y} {z}")
            return nwpc, False

    def CheckReturn(self, wpc,mission):  # check it this waypoint is valid 
        MidDelay=0
        if self.wp!=wpc:
            MidDelay=(self.DisMatrix[self.wp].get(wpc))/self.drone.speed+self.drone.loiter
        else:
            MidDelay=self.drone.loiter
        time, _=self.ShortUpload.get(wpc)
        Delay=MidDelay+time
        if self.EndTime-self.time<Delay:
            return True
        if len(mission)>0:
            #if self.time+MidDelay<mission[1] and self.time+Delay>mission[1]:
            if (mission[1] -self.time)<Delay:      # Maybe storage is empty, but go to  waypoint out of connection
                return True
        for m, tt in self.Mission_SR.items():
            if len(tt)>0 and (self.Mission_DL[m] -self.time)<Delay: # If drone has storage, then check 
                #print(f"see should {m} {self.Mission_DL[m]} {self.time} {Delay} ")
                #print(f"Need to return {Delay} < {self.Mission_DL[m]-self.time}")
                #tt, nwpc=self.ShortUpload.get(self.wp) # self.wp must be not connected, as we have 
                #print(f"see this time {tt} ") 
                #print(f" see DL {[x[0] for x in  list(self.Mission_SR.items()) if len(x[1])>0]}")

                return True
        return False
    def RelexCheckReturn(self, wpc,mission):  # check it this waypoint is valid 
        MidDelay=0
        if self.wp!=wpc:
            MidDelay=(self.DisMatrix[self.wp].get(wpc))/self.drone.speed+self.drone.loiter
        else:
            MidDelay=self.drone.loiter
        time, _=self.ShortUpload.get(wpc)
        Delay=MidDelay+time
        if self.EndTime-self.time<Delay:
            return True
        if len(mission)>0:
            if self.time+MidDelay<mission[1] and self.time+Delay>mission[1]:
            #if (mission[1] -self.time)<Delay:      # Maybe storage is empty, but go to  waypoint out of connection
                return True
        for m, tt in self.Mission_SR.items():
            if len(tt)>0 and (self.Mission_DL[m] -self.time)<Delay: # If drone has storage, then check 
                #print(f"see should {m} {self.Mission_DL[m]} {self.time} {Delay} ")
                #print(f"Need to return {Delay} < {self.Mission_DL[m]-self.time}")
                #tt, nwpc=self.ShortUpload.get(self.wp) # self.wp must be not connected, as we have 
                #print(f"see this time {tt} ") 
                return True
        return False

    def DL_Scheduling(self,EndTime, Improve): 
        #self.DrawWPCs()
        self.EndTime=EndTime
        self.waypointSeq.append(self.wp)
        ########Define a NOWPCGrid 
        Reward=0; P=0
        self.initialTask()
        self.ClusterTask()
        st=time.time()
        while self.time<=EndTime:
            dlist=list(self.Mission_DL.items())  # Check the dl! 
            dlist.sort(key=lambda s: s[1]) # Get the earliest deadline. 
            cdl=dlist[0][1]
            Gridset=[]; tmp=[]
            for mm, dlv in dlist: 
                if dlv==cdl:
                    tmp=tmp+(list(self.Mission_Grid[mm]))
                else:
                    Gridset.append(tmp)
                    cdl=dlv
                    tmp=list(self.Mission_Grid[mm])
            Gridset.append(tmp)
            count=0; NoUpDL=True
            while count<len(Gridset) and NoUpDL and self.time<=EndTime:
                CurGrids=copy.copy(Gridset[count])
                #print(f"check  Gridset {count} {CurGrids}")
                CurGrids=[g for g in CurGrids if len(self.Grid_toCov[g])>0 ]
                #print(f" what left {CurGrids}")
                #print(f"Drone {self.drone.id} Enter task {Gridset[count]} at {self.time}")
                if len(CurGrids)>0:
                    g_num=0
                    while g_num< len(CurGrids) and (NoUpDL and self.time<=EndTime):
                        near_g=CurGrids[g_num] 
                        #near_g=CurGrids[-1] 
                        New=True
                        SugH=0
                        #print(f"If Enter {near_g} {len(self.Grid_toCov[near_g])}")
                        while len(self.Grid_toCov[near_g])>0 and (NoUpDL and self.time<=EndTime): # This while loop is not used, because now there is only one item in it. 
                            areas=self.Grid_toCov[near_g]
                            #print(f"Enter {near_g} {self.time} {len(areas)}")
                            #nwpc, NoWPCs=self.BestTopDownNeigWPC(areas, near_g) # Here near_g is 
                            #nwpc, NoWPCs=self.RewardNeigWPC(areas, near_g) # Here near_g is
                            #nwpc, NoWPCs=self.BestCovNeig    WPC(areas, near_g) # Here near_g is
                            if New:
                                SugH,_,Ctime,Wseq, NoWPCs=self.AdjustHeightNeigWPC(areas, near_g) # Here near_g is 
                                #print(f"Check {self.time+Ctime} {NoWPCs}")
                                #print(f"{near_g} Check if {NoWPCs} {Wseq} {self.time} ")
                                New=False
                            if not NoWPCs and len(Wseq)>0:
                                for nwpc in Wseq:
                                    self.waypointSeq.append(nwpc)
                                    #planner.DrawWPsequence()
                                    reward,SReward,Penalty,NoUpDL=self.ClusterStateTrans(nwpc)
                                    Reward=Reward+reward
                                    P=P+Penalty
                                    if not NoUpDL:
                                        #print(f"Jump out AjustHeight")
                                        break
                            break
                        g_num=g_num+1
                count=count+1
            if not Improve and count==len(Gridset) and NoUpDL: 
                nwpc=self.ShortUpload.get(self.wp)[1]
                self.waypointSeq.append(nwpc)
                #planner.DrawWPsequence()
                reward,SReward,Penalty,NoUpDL=self.ClusterStateTrans(nwpc)
                Reward=Reward+reward
                P=P+Penalty
            ###################################### Enter improve phase!!!!!!!!!!!! 
            if Improve and count==len(Gridset) and NoUpDL:# Now, there is no NoUPLOAD! 
                SRlength=max([len(k) for k in self.Mission_SR.values()])
                if SRlength==0: # Means drone storage is empty.
                    count=0
                    while count<len(Gridset)and NoUpDL and self.time<=EndTime: #Enter improve phase 
                        CurGrids=copy.copy(Gridset[count])
                        g_num=0
                        while g_num< len(CurGrids) and (NoUpDL and self.time<=EndTime):
                            near_g=CurGrids[g_num] 
                            #near_g=CurGrids[0] 
                            #print(f"{self.drone.id} to improve  {near_g} at {self.time} ")
                            while NoUpDL and self.time<=EndTime:
                                nwpc, NoWPCs=self.ImproveRewardWPC(self.Fix_toCov[near_g],near_g) # Here near_g is 
                                #print(f"{near_g} if improve {NoWPCs} {self.time} ")
                                if NoWPCs:
                                    #print(f"Drone {self.drone.id} {count} No Improve at task {near_g} at {self.time}")
                                    break
                                self.waypointSeq.append(nwpc)
                                reward,SReward,Penalty,NoUpDL=self.ClusterStateTrans(nwpc)
                                #print(f"Drone {count} {self.drone.id} improve {near_g} {reward} {SReward} {Penalty} {NoUpDL} {self.time} ")
                                Reward=Reward+reward
                                P=P+Penalty
                            g_num=g_num+1
                        count=count+1
                if count==len(Gridset) and NoUpDL:
                    #print(f" To upload Data")
                    nwpc=self.ShortUpload.get(self.wp)[1]
                    self.waypointSeq.append(nwpc)
                    #planner.DrawWPsequence()
                    reward,SReward,Penalty,NoUpDL=self.ClusterStateTrans(nwpc)
                    Reward=Reward+reward
                    P=P+Penalty
            #planner.DrawWPsequence()
        if IFLOG:
            self.logfile.write(f"Time {self.drone.id} {time.time()-st}\n")
            self.logfile.write(f"Total {self.drone.id} {Reward} {P} {len(self.waypointSeq)} {self.waypointSeq}\n")
        # print(f"Planning time {time.time()-st} ")    
        # print(f" drone {self.drone.id} waypoints {Reward}, {P}, {len(self.waypointSeq)} {self.waypointSeq}")
        return Reward, P, time.time()-st
    
    def DL_Comp(self,EndTime, which): 
        self.EndTime=EndTime
        self.waypointSeq.append(self.wp)
        Reward=0; P=0
        self.initialTask()
        self.ClusterTask()
        st=time.time()
        while self.time<=EndTime:
            dlist=list(self.Mission_DL.items())  # Check the dl! 
            dlist.sort(key=lambda s: s[1]) # Get the earliest deadline. 
            cdl=dlist[0][1]
            Gridset=[]; tmp=[]
            for mm, dlv in dlist: 
                if dlv==cdl:
                    tmp=tmp+(list(self.Mission_Grid[mm]))
                else:
                    Gridset.append(tmp)
                    cdl=dlv
                    tmp=list(self.Mission_Grid[mm])
            Gridset.append(tmp)
            count=0; NoUpDL=True
            while count<len(Gridset) and NoUpDL and self.time<=EndTime:
                CurGrids=copy.copy(Gridset[count])
                CurGrids=[g for g in CurGrids if len(self.Grid_toCov[g])>0]
                if len(CurGrids)>0:
                    near_g=CurGrids[0] 
                    while len(self.Grid_toCov[near_g])>0 and (NoUpDL and self.time<=EndTime): # This while loop is not used, because now there is only one item in it. 
                        areas=self.Grid_toCov[near_g]
                        if which==3:
                            nwpc, NoWPCs=self.DistanceNeigWPC(areas, near_g) # Here near_g is 
                        if which==4:
                            nwpc, NoWPCs=self.RewardNeigWPC(areas,near_g) # Here near_g is 
                        if NoWPCs:  
                            nwpc=self.ShortUpload.get(self.wp)[1]
                            self.waypointSeq.append(nwpc)
                            reward,SReward,Penalty,NoUpDL=self.ClusterStateTrans(nwpc)
                            Reward=Reward+reward
                            P=P+Penalty
                            break
                        else:
                            self.waypointSeq.append(nwpc)
                            reward,SReward,Penalty,NoUpDL=self.ClusterStateTrans(nwpc)
                            Reward=Reward+reward
                            P=P+Penalty
                count=count+1
            if count==len(Gridset) and NoUpDL: 
                nwpc=self.ShortUpload.get(self.wp)[1]
                self.waypointSeq.append(nwpc)
                reward,SReward,Penalty,NoUpDL=self.ClusterStateTrans(nwpc)
                Reward=Reward+reward
                P=P+Penalty
        self.logfile.write(f"Time {self.drone.id} {time.time()-st}\n")
        self.logfile.write(f"Total {self.drone.id} {Reward} {P} {len(self.waypointSeq)} {self.waypointSeq}\n")
        # print(f"Planning time {time.time()-st}")    
        # print(f" drone {self.drone.id}  waypoints {Reward}, {P}, {len(self.waypointSeq)} {self.waypointSeq}")
        return Reward, P, time.time()-st
    
    def Comp_Scheduling(self,EndTime, which): 
        #self.DrawWPCs()
        self.EndTime=EndTime
        self.waypointSeq.append(self.wp)
        Reward=0; P=0
        #print(f" length", len(self.WPCHeight))   # WPCs' key is the height: waypoints; Cover_map's key is waypoint: areas
        self.initialTask()
        self.ClusterTask()
        st=time.time()
        while self.time<=EndTime:
            #return Reward, Ttime, Covercount
            WPCs=[w for w in self.WPCs if not (self.CheckReturn(w,[]))]
            if len(WPCs)==0:
                #print(f"see Upload")
                nwpc=self.ShortUpload.get(self.wp)[1]
            elif which==1:#Distance_Driven 
                # Covlist=[x for x in Covlist if x[2]>0]
                Covlist=[(self.GetDataReward(w),w) for w in WPCs]
                Covlist=[x for x in Covlist if x[0][0]>0]
                if len(Covlist)==0:
                    #print(f"see Upload")
                    nwpc=self.ShortUpload.get(self.wp)[1]
                else:
                    CoWPCs=[x[1] for x in Covlist]
                    Covlist=[x[0] for x in Covlist]
                    x,y,z=min(Covlist, key=lambda x: x[1])
                    adex=Covlist.index((x,y,z))
                    nwpc=CoWPCs[adex] 
            elif which==2:  
                Covlist=[self.GetDataReward(w) for w in WPCs]
                x,y,z=max(Covlist, key=lambda x: x[0]/x[1])
                if x==0:
                    #print(f"see Upload")
                    nwpc=self.ShortUpload.get(self.wp)[1]
                else:
                    adex=Covlist.index((x,y,z))
                    nwpc=WPCs[adex]
            self.waypointSeq.append(nwpc)
            reward,SReward,Penalty,NoUpDL=self.ClusterStateTrans(nwpc)
            Reward=Reward+reward
            P=P+Penalty
        if IFLOG:
            self.logfile.write(f"Time {self.drone.id} {time.time()-st}\n")
            self.logfile.write(f"Total {self.drone.id} {Reward} {P} {len(self.waypointSeq)} {self.waypointSeq}\n")
        return Reward, P, time.time()-st
    
    def DL_Release_FP(self,EndTime, FPnum, Improve): 
        #self.DrawWPCs()
        self.EndTime=EndTime
        self.waypointSeq.append(self.wp)
        ########Define a NOWPCGrid 
        Reward=0; P=0
        self.initialTask()
        self.ClusterTask()
        st=time.time()
        while self.time<=EndTime:
            dlist=list(self.Mission_DL.items())  # Check the dl! 
            dlist.sort(key=lambda s: s[1]) # Get the earliest deadline. 
            cdl=dlist[0][1]
            Gridset=[]; tmp=[]
            for mm, dlv in dlist: 
                if dlv==cdl:
                    tmp=tmp+(list(self.Mission_Grid[mm]))
                else:
                    Gridset.append(tmp)
                    cdl=dlv
                    tmp=list(self.Mission_Grid[mm])
            Gridset.append(tmp)
            count=0; NoUpDL=True
            while count<len(Gridset) and NoUpDL and self.time<=EndTime:
                CurGrids=copy.copy(Gridset[count])
                CurGrids=[g for g in CurGrids if len(self.Grid_toCov[g])>0 ]
                if len(CurGrids)>0:
                    #print(f"Enter CurGrids {CurGrids}")
                    while sum([len(self.Grid_toCov[g]) for g in CurGrids])>0 and (NoUpDL and self.time<=EndTime): # This while loop is not used, because now there is only one item in it. 
                        if len(CurGrids)>1:
                            nwpc, NoWPCs=self.MultipleCovWPC(CurGrids) # Here near_g is
                            if NoWPCs:
                                break
                            self.waypointSeq.append(nwpc)
                            reward,SReward,Penalty,NoUpDL=self.ClusterStateTrans(nwpc)
                            Reward=Reward+reward
                            P=P+Penalty
                        else:
                            near_g=CurGrids[0]
                            areas=self.Grid_toCov[near_g]
                            SugH,_,Ctime,Wseq, NoWPCs=self.AdjustHeightNeigWPC(areas, near_g,FPnum) # Here near_g is 
                            if not NoWPCs and len(Wseq)>0:
                                for nwpc in Wseq:
                                    self.waypointSeq.append(nwpc)
                                    #planner.DrawWPsequence()
                                    reward,SReward,Penalty,NoUpDL=self.ClusterStateTrans(nwpc)
                                    Reward=Reward+reward
                                    P=P+Penalty
                                    if not NoUpDL:
                                        #print(f"check! ")
                                        break
                            break 
                count=count+1
            if not Improve and count==len(Gridset) and NoUpDL: 
                nwpc=self.ShortUpload.get(self.wp)[1]
                self.waypointSeq.append(nwpc)
                #planner.DrawWPsequence()
                reward,SReward,Penalty,NoUpDL=self.ClusterStateTrans(nwpc)
                Reward=Reward+reward
                P=P+Penalty
            ###################################### Enter improve phase!!!!!!!!!!!! 
            if Improve and count==len(Gridset) and NoUpDL:# Now, there is no NoUPLOAD! 
                SRlength=max([len(k) for k in self.Mission_SR.values()])
                if SRlength==0: # Means drone storage is empty.
                    count=0
                    while count<len(Gridset)and NoUpDL and self.time<=EndTime: #Enter improve phase 
                        CurGrids=copy.copy(Gridset[count])
                        #print(f"improve {CurGrids}")
                        while NoUpDL and self.time<=EndTime:
                            nwpc, NoWPCs=self.MultipleRewardWPC(CurGrids) # Here near_g is
                            #nwpc, NoWPCs=self.MultipleRewardWPC(self.Fix_toCov[near_g],near_g) # Here near_g is 
                            if NoWPCs:
                                #print(f"Drone {self.drone.id} {count} No Improve at task {near_g} at {self.time}")
                                break
                            self.waypointSeq.append(nwpc)
                            reward,SReward,Penalty,NoUpDL=self.ClusterStateTrans(nwpc)
                            #print(f"Drone {count} {self.drone.id} improve {near_g} {reward} {SReward} {Penalty} {NoUpDL} ")
                            Reward=Reward+reward
                            P=P+Penalty
                        count=count+1
                if count==len(Gridset) and NoUpDL:
                    #print(f" To upload Data")
                    nwpc=self.ShortUpload.get(self.wp)[1]
                    self.waypointSeq.append(nwpc)
                    #planner.DrawWPsequence()
                    reward,SReward,Penalty,NoUpDL=self.ClusterStateTrans(nwpc)
                    Reward=Reward+reward
                    P=P+Penalty
            #planner.DrawWPsequence()
        if IFLOG:
            self.logfile.write(f"Time {self.drone.id} {time.time()-st}\n")
            self.logfile.write(f"Total {self.drone.id} {Reward} {P} {len(self.waypointSeq)} {self.waypointSeq}\n")
        #print(f"Planning time ")    
        print(f" drone {self.drone.id} waypoints {Reward}, {P}, {len(self.waypointSeq)} {self.waypointSeq}")
        return Reward, P, time.time()-st

IFLOG=True

def AllComp(TANum,GWP,FPnum,Drones,init, Plantime,inloc,GCloc, Missions,DecomposeSize,EFA, Res,tasks,log=''):
    if IFLOG:
        log.write(f"Start {TANum} {GWP} {FPnum}\n")
    AreaPara=Decomposition(DecomposeSize,Res,tasks) # Need put tasks there!!!
    Drones,Bidders=Auction_Comp(TANum, AreaPara, Drones, Res,Missions,3, EFA,GCloc)
    Rewardlist=[]
    Penaltylist=[]
    Runtimelist=[]
    LogTask=[]
    LogMiss=[]
    LogReward=[]
    for drone in Drones:
        planner=FlightPlanner(drone,init=init, iniloc=inloc, GCloc=inloc,Missions=Missions,Res=Res) # Here, GCloc and iniloc divided by Res!
        planner.logfile=log
        if GWP==1:
            planner.GenWaypoint_SetCover()
        if GWP==2:
            planner.GenWaypoint_Adjust()
        if GWP==3:
            planner.GenWaypoint_Regular()
        if FPnum==0:
            Reward, P, Runtim=WPsequence=planner.DL_Scheduling(Plantime, Improve=True)
        elif FPnum in [1,2]:
            Reward, P, Runtim=planner.Comp_Scheduling(Plantime,FPnum)
        elif FPnum in [3,4]:
            Reward, P, Runtim=planner.DL_Comp(Plantime, FPnum)
        elif FPnum in [5,6]:
            Reward, P, Runtim=planner.DL_Release_FP(Plantime, FPnum,Improve=True )
        LogTask.append(planner.logtasks)
        LogMiss.append(planner.logmiss)
        LogReward.append(planner.logReward)
        Rewardlist.append(Reward)
        Penaltylist.append(P)
        Runtimelist.append(Runtim)
        #print(f"Reward {Reward} P {P}")
        #planner.DrawWPsequence()
    print(f"{TANum} {GWP} {FPnum} {sum(Rewardlist)} {sum(Penaltylist)} ")
    return Rewardlist, Penaltylist, Runtimelist,LogTask,LogMiss,LogReward


if __name__ == "__main__":
    Missions=defaultdict(dict)  #  Missions: id, period, priority, 
    Missions['BM']=[ 10, 2]  #Other area 
    Missions['FI']=[ 5, 1]  #track the fire 
    Missions['FT']=[ 2.5, 3]  # track the fire perimeter and the risky area (arrival within 10 min)
    #Missions['FL']=[ 3, 2]  # FL is tracking the fire perimeter.
    #Missions['FD']=[ 2, 3]
    ################ Get tasks.
    #dir='/home/fangqiliu/eclipse-workspace_Python/Drone_path/CoveragePathPlanning-master/farsite'
    dir='../../../farsite'
    foldername='FQ_sim'
    # logfile=f"logtask.txt"
    # if IFLOG:
    #     logfile=open(logfile, "w")    
    DecomposeSize=50
    #Bursite=(702460.0,4309700.0,703540.0,4310820.0 )
    Bursite=(702460.0,4309700.0,702860.0,4310200.0 )
    Res=10
    ########################## Do decomposition~ 
    file='CARB_BurnUnits/CARB_BurnUnits.shp'
    data=gpd.read_file(f"{dir}/{file}")
    Bardata=ClearBarrier(data)
    wind=15
    #CreateDyRxfire(Bardata,fir_name,dir, [2],wind=wind)
    fir_name=f"FQ_Sim_{wind}"
    STtime=40
    EFA,EFAdict,bound =GetFirSim(Bardata,  foldername, fir_name, dir, Bursite, Res=Res,time=STtime,wind=wind)                  
    TM=TaskManager(Missions)
    print(f"Start Task Generation")
    tasks=TM.DeclareGridEFA(EFA,init=0)
    ########################################
    sensorfile='Data/sensor_info.csv'
    PPMfile='Data/PPM_table.csv'
    DroneNum=6
    speeds=[5,5,5,5,3,5]
    loiter=[1,2,1,1,1,2]
    ranges=[300,200,500,300,300,500]
    GCloc=(0,500)
    GCloc=(0,0)
    inloc=(0,0,0)
    sensgrp=[['ZENMUSE_XT2_t','ZENMUSE_XT2_r'],['DJI_Air2S'],['ZENMUSE_XT2_t','ZENMUSE_XT2_r'],['DJI_Air2S'],['ZENMUSE_XT2_t','ZENMUSE_XT2_r'],['ZENMUSE_XT2_t','ZENMUSE_XT2_r']]
    #sensgrp=[['ZENMUSE_XT2_t','ZENMUSE_XT2_r'],['DJI_Air2S'],['ZENMUSE_XT2_t','ZENMUSE_XT2_r'],['ZENMUSE_XT2_t','ZENMUSE_XT2_r']]
    Drones=LoadDrones(sensorfile,PPMfile,DroneNum, speeds, sensgrp, Res,loiter,ranges)
     ########################## Do decomposition~  1: Normal 2: Inter 3: Area 4: Voronoi
    # SimTime=2*60*60 #(seconds for plan 6 times 
    logfile=f"./Results/try_{wind}_{STtime}"
    log=''
    if IFLOG:
        log=open(logfile, "w")
    init=0; Plantime=60*20
    TANum=4;GWP=1;FPnum=5
    AllComp(TANum,GWP,FPnum,Drones,init, Plantime,inloc,GCloc, Missions,DecomposeSize,EFA, Res,tasks,log=log)
    if IFLOG:
        log.close()
    #Drones,Bidders=Auction_Comp(TANum, AreaPara, Drones, Res,Missions,3, EFA,GCloc)
    #print(f"see Drones cost {[sum(list(Bidders[i].Tak_UT.values())) for i in range(len(Bidders))]}")
    #TrackDraw(Drones, EFA)
    ################################# GWP= 1: WPC_SetCover; 2: WPC; 3: Regular
    #######PLAN 1: DD+Return; 2: Reward_Driven+Return; 3: DL+DD+Return; 4: DL+RD+Return; 5: DL+CO+Return; 6: DL+CO+NoReturn 
    
    
    # for drone in Drones:
    #     planner=FlightPlanner(drone,init=init, iniloc=inloc, GCloc=inloc,Missions=Missions,Res=Res) # Here, GCloc and iniloc divided by Res!
    #     if GWP==1:
    #         planner.GenWaypoint_SetCover()
    #     if GWP==2:
    #         planner.GenWaypoint_Adjust()
    #     if GWP==3:
    #         planner.GenWaypoint_Regular()
    #     if which==0:
    #         WPsequence=planner.DL_Scheduling(Plantime, Improve=True)
    #     elif which in [1,2]:
    #         WPsequence=planner.Comp_Scheduling(Plantime,which)
    #     elif which in [3,4]:
    #         WPsequence=planner.DL_Comp(Plantime, which)
    #     planner.DrawWPsequence()
    #logfile.close()
    
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

    # def TrackState(self):
    #     print(f" DroneID: {self.drone.id}")
    #     print(f" waypoint: {self.wp}")
    #     print(f" time: {self.time}")


        # def WCPCosArea(self, areas,g):   # Too simple, To get the waypoint cover the closet areas 
    #     Diss=[self.Distance(self.wp, (a[0],a[1],0)) for a in areas] #Here the distance can be compute less????
    #     adex=Diss.index(min(Diss))
    #     near_a=areas[adex]
    #     to_wp=self.Area_WPC.get(near_a)
    #     poheights=list(self.drone.WPCinfo.get(g[0]).keys())
    #     wps=[w for w in to_wp if w[2] in poheights]   # why did not match????? 
    #     wps.sort(key=lambda s: s[2], reverse=True)
    #     Return=self.CheckReturn(wps[0],((g[0],g[1]),self.Mission_DL[g[0],g[1]]))
    #     #nwpc=wps[0]
    #     if Return:
    #         nwpc=self.ShortUpload.get(self.wp)[1]
    #     else:
    #         nwpc=wps[0]
    #     return nwpc
        # def GenWaypoint_None(self):
    #     Tak_H_FOV={}
    #     allh=[]
    #     for tak in list(self.drone.MissionSet):
    #         heights=self.drone.WPCinfo.get(tak)
    #         tmph=[[h, heights[h][0]] for h in heights]
    #         tmph=sorted(tmph, key=lambda l:l[0], reverse=False) 
    #         tst=[];delset=[]
    #         for t in tmph:
    #             if t[1] not in tst:
    #                 tst.append(t[1])
    #             else:   ############ It is okay, because when I combine that, I guarantee h<<, cov<<
    #                 delset.append(t)
    #         for d in delset:
    #             tmph.remove(d)
    #         #print(f"what tmph it get? {tmph}")
    #         allh=allh+[i[0] for i in tmph] ## Get all heights for Grids. 
    #         Tak_H_FOV[tak]=tmph
    #         self.WPC_TakFov[tak]=tmph
    #
    #     #print(f"see Tak_H_FOV {Tak_H_FOV}")
    #     # for k in 
    #     x0,y0,z=np.array(self.GCloc)
    #     px=[i for i in range(x0-self.drone.range//self.Res,x0+self.drone.range//self.Res) if i>=0 and i%(2)==0]
    #     #print(f"allh", px, allh, min(allh), max(allh))
    #     allz= list(range(int(min(allh)), int(max(allh)),self.Res*2))+[ int(max(allh))]
    #     #print(f" allz {allz}")
    #     yrange=[y0-self.drone.range, y0+self.drone.range ]
    #     upwpc=set()
    #     for x in px:
    #         for z in allz:
    #             tmp=(self.drone.range)**2-z**2-((x-x0)*self.Res)**2
    #             if tmp>=0:
    #                 y1=(np.sqrt(tmp))//self.Res+y0
    #                 y2= y0-(np.sqrt(tmp))//self.Res 
    #                 upwpc.add((x,int(y1),z))
    #                 upwpc.add((x,int(y2),z))
    #     UW=list(upwpc)
    #     ##########################################################
    #     for grid in self.tasks:#Get the minimum and maximum x
    #         areas=grid[3]
    #         tak=grid[0]
    #         for a in areas:
    #             self.Area_Task[a[0], a[1]].append((tak,a[2]))
    #             self.Area_grid[a[0], a[1]]=(0,0)#grid[1])   #################### FQFQ update
    #             #self.Area_grid[a[0], a[1]]=(grid[1])   #################### FQFQ update
    #         xal=[x[0] for x in areas]
    #         yal=[x[1] for x in areas]
    #         tmph=Tak_H_FOV[tak]
    #         for wpc in tmph:
    #             fov=wpc[1]/(self.Res)####### Already divided by Res!!!
    #             x=min(xal)
    #             while x<=max(xal):
    #                 y=min(yal)
    #                 while y<=max(yal):
    #                     self.WPCHeight[tak,wpc[0]].add((x+fov/2,y+fov/2,wpc[0]))    
    #                     self.WPCs.append((x+fov/2,y+fov/2,wpc[0]))
    #                     conn=(self.Distance((x+fov/2,y+fov/2,wpc[0]), self.GCloc)<=self.drone.range)
    #                     self.Connection[x+fov/2,y+fov/2,wpc[0]]=conn
    #                     ################################### NO Connection! 
    #                     if conn==True:
    #                         self.Con_wpc.append((x+fov/2,y+fov/2,wpc[0]))
    #                         self.ShortUpload[(x+fov/2,y+fov/2,wpc[0])]=(self.drone.loiter,(x+fov/2,y+fov/2,wpc[0])) # if is connected, it is itself. 
    #                     else:
    #                         Dis=[self.Distance((x+fov/2,y+fov/2,wpc[0]), ww) for ww in UW]
    #                         wm=Dis.index(min(Dis))
    #                         Ttime=min(Dis)/self.drone.speed+self.drone.loiter # maybe 1 s
    #                         self.ShortUpload[(x+fov/2,y+fov/2,wpc[0])]=(Ttime,UW[wm])
    #                         self.WPCs.append(UW[wm])
    #                         self.Connection[UW[wm]]=True
    #                         self.ShortUpload[UW[wm]]=(self.drone.loiter,UW[wm])
    #                         #UsedUW.add(UW[wm])
    #                     ########################### For get the coverage area
    #                     cx=int(x)
    #                     while cx<x+fov:
    #                        cy=int(y) 
    #                        while cy<y+fov:
    #                             if self.Cover_map.get((x+fov/2,y+fov/2,wpc[0]))==None:
    #                                 self.Cover_map[x+fov/2,y+fov/2,wpc[0]].append((cx,cy,tak))
    #                                 self.Area_WPC[(cx,cy),tak].append((x+fov/2,y+fov/2,wpc[0]))
    #                             elif (cx,cy,tak) not in self.Cover_map[x+fov/2,y+fov/2,wpc[0]]:
    #                                 self.Cover_map[x+fov/2,y+fov/2,wpc[0]].append((cx,cy,tak))
    #                                 self.Area_WPC[(cx,cy),tak].append((x+fov/2,y+fov/2,wpc[0]))
    #                             cy=cy+1
    #                        cx=cx+1
    #                     ########################
    #                     y=y+fov
    #                 x=x+fov
    #     st=time.time()
    #     self.WPCs=list(set(self.WPCs))
    #     for i in range(len(self.WPCs)):
    #         for j in range(i+1,len(self.WPCs)):
    #             self.DisMatrix[self.WPCs[i]][self.WPCs[j]]=self.Distance(self.WPCs[i], self.WPCs[j])
    #             self.DisMatrix[self.WPCs[j]][self.WPCs[i]]=self.DisMatrix[self.WPCs[i]][self.WPCs[j]]
    #     print(f"time to compute distance {time.time()-st}")
    # def SetCoveWPC_old(self):
    #     for key, wpc in self.WPCHeight.items():
    #         mm,h=key
    #         #sorted(list(wpc))
    #         wps = sorted(list(wpc), key=lambda x: x[0])
    #         cuwpcs=[wps[0]]
    #         #cuwpcs=[]
    #         alltak=copy.copy(self.drone.Tak_Area[mm])
    #         #print(f"why {set(self.Cover_map[wps[0]])}")
    #         tmparea=alltak-set([a[:-1] for a in self.Cover_map[wps[0]]])
    #         #print(f"len {len(alltak)} {len(tmparea)}")
    #         wps.remove(wps[0])
    #         while len(tmparea)>0:
    #             #print([[a[:-1] for a in self.Cover_map[w]] for w in wps])
    #             cov=[(set([a[:-1] for a in self.Cover_map[w]]).intersection(tmparea),w) for w in wps]
    #             #print(f"fail1", cov)
    #             sotcov=sorted(list(cov), key=lambda x: len(x[0]), reverse=True)
    #             #print(f"fail", sotcov,  tmparea)
    #             cuwpcs.append(sotcov[0][1])
    #             wps.remove(sotcov[0][1])
    #             tmparea=tmparea-sotcov[0][0]
    #         self.WPCs=self.WPCs+cuwpcs 
    #     st=time.time()
    #     self.WPCs=list(set(self.WPCs))
    #     for i in range(len(self.WPCs)):
    #         for j in range(i+1,len(self.WPCs)):
    #             self.DisMatrix[self.WPCs[i]][self.WPCs[j]]=self.Distance(self.WPCs[i], self.WPCs[j])
    #             self.DisMatrix[self.WPCs[j]][self.WPCs[i]]=self.DisMatrix[self.WPCs[i]][self.WPCs[j]]
    #     print(f"time to compute distance {time.time()-st}")


