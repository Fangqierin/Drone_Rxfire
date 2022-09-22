import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from bcd_helper import imshow, imshow_scatter
import math
from collections import defaultdict
import random
import time
import geopandas as gpd
from FQ_TaskAllocation import  Auction, TrackDraw
from FQ_Drones_Info import Sensor, Drone, ReadSen_PPM,LoadDrones
from FQ_Task_Generator import  TaskManager 
from FQ_Firesim import DyFarsitefile, ClearBarrier, CreateDyRxfire
from FQ_TG_client import GetFirSim, Decomposition, DrawTask
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
    def __init__(self, Drone, init=0, planTime=10, iniloc=[], GCloc=[], endloc=[], Missions={}, Res=10, DecomposeSize=200):
        self.drone=Drone
        self.tasks=Drone.area
        self.GCloc=GCloc
        self.Res=Res
        self.MissionDict=Missions
        self.cw=0
        self.time=init
        self.end=(init+planTime)*60
        self.wp=iniloc
        self.endloc=endloc
        self.TaskState=defaultdict(dict)# Deadline, Reward, stored reward, Acum. Penalty
        self.WPCHeight=defaultdict(list)
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

        #Then I need to the the location of the GC!!!
    def GenWaypoint(self):
        ####old  task Grids.append([mm, key, len(AreaPara[mm][key])*(Res**2),AreaPara[mm][key],period])
        ###Current task: Grids.append([mm, cell, len(grids)*(Res**2),grids,period,(cx,cy)])
        #Area[(xc,yc)].append([x[i],y[i],mm[1][0]]) #initial time
        #AreaPara[mm[0]]=Area
        #Taskdict['BM']=[init, BMA, Missions['BM'][0],Missions['BM'][1]]
        #task is drone.area
        print(f"check", self.drone.range)
        print(f"GCloc {self.GCloc}")
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
            #print(f"what tmph it get? {tmph}")
            allh=allh+[i[0] for i in tmph] ## Get all heights for Grids. 
            Tak_H_FOV[tak]=tmph
            
        print(f"see Tak_H_FOV {Tak_H_FOV}")
        # for k in 
        x0,y0,z=np.array(self.GCloc)
        px=[i for i in range(x0-self.drone.range//self.Res,x0+self.drone.range//self.Res) if i>=0 and i%(2)==0]
        #print(f"allh", px, allh, min(allh), max(allh))
        allz= list(range(int(min(allh)), int(max(allh)),self.Res*2))+[ int(max(allh))]
        #print(f" allz {allz}")
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
                    #print(f"get {x,int(y1),z} {x,int(y2),z} {self.Distance(upw, self.GCloc)} {self.Distance(upw2, self.GCloc)}")
        UW=list(upwpc)
        #fig = plt.figure()
        # ax = fig.add_subplot(projection='3d')
        # ax.scatter([i[0] for i in UW],[i[1] for i in UW],[i[2] for i in UW] )
        # plt.show()
        #UsedUW=set() # record which UW is selected 
        ##########################################################
        for grid in self.tasks:#Get the minimum and maximum x
            areas=grid[3]
            tak=grid[0]
            for a in areas:
                self.Area_Task[a[0], a[1]].append((tak,a[2]))
                self.Area_grid[a[0], a[1]]=(0,0)#grid[1])   #################### FQFQ update
                #self.Area_grid[a[0], a[1]]=(grid[1])   #################### FQFQ update

                #print(grid)
            xal=[x[0] for x in areas]
            yal=[x[1] for x in areas]
            tmph=Tak_H_FOV[tak]
            for wpc in tmph:
                fov=wpc[1]/(self.Res)####### Already divided by Res!!!
                x=min(xal)
                while x<=max(xal):
                    y=min(yal)
                    while y<=max(yal):
                        self.WPCHeight[wpc[0]].append((x+fov/2,y+fov/2,wpc[0]))
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
                            #print(f"see Get the {UW[wm]}")
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
                        y=y+fov
                    x=x+fov
        # for ww in UsedUW:
        #     ww=ww[:-1]
        #     print(f"see cov  {ww}", self.Area_grid.get(ww))
        #     if self.Area_grid.get(ww)!=None:
        #        print(f"see cov ", self.Area_grid.get(ww))
        st=time.time()
        self.WPCs=list(set(self.WPCs))
        for i in range(len(self.WPCs)):
            for j in range(i+1,len(self.WPCs)):
                self.DisMatrix[self.WPCs[i]][self.WPCs[j]]=self.Distance(self.WPCs[i], self.WPCs[j])
                self.DisMatrix[self.WPCs[j]][self.WPCs[i]]=self.DisMatrix[self.WPCs[i]][self.WPCs[j]]
        print(f"time to compute distance {time.time()-st}")
        # for w in list(set(self.WPCs)-set(self.Con_wpc)):
        #     Dis=[self.DisMatrix[w][ww] for ww in self.Con_wpc]
        #     wm=Dis.index(min(Dis))
        #     Ttime=min(Dis)/self.drone.speed+self.drone.loiter # maybe 1 s
        #     self.ShortUpload[w]=(Ttime,self.Con_wpc[wm])
        # for w in self.Con_wpc:
        #     self.ShortUpload[w]=(self.drone.loiter,w) # if is connected, it is itself. 
        print(f"time to compute short upload {time.time()-st}")
    def initialTask(self):     #Here we only consider to handle started task!
        print(f"see Gird_Task {self.Area_Task}")
        for a in self.Area_Task:
            for t in self.Area_Task[a]: 
                # Here t is (task, (rt, et)) 
                offset= (max(t[1][0],self.time)-self.time)%((self.MissionDict[t[0]][0]))*60
                
                #Here we have release time and ending time!!!!! 
                self.TaskState[a][t[0],offset]=(0,0)# Deadline job reward, stored reward. 
                
                self.Area_Mission_time[a][t[0],offset]=(t[1][0]*60, t[1][1]*60 if t[1][1]!=-1 else -1)
        ########### Here initialTask have problem, 
        #default is 0,0 but it is not! 
    def ClusterTask(self):# cluster tasks based on their mission
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
                    #print(f"see {(mission,offset)} {list(tmp_jr.keys())}")
                    tmp_jr[mission,offset]=[]
                if (mission,offset) not in list(tmp_sr.keys()):
                    tmp_sr[mission,offset]=[]
                if rt<=self.time : ############################# Add a release time judgement! 
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
        #print(f"whyyyy? check {self.Grid_toCov} ")

    def CheckUpdateTask(self, time): # Add/remove task which start after initial time. 
        pass
    def CheckReleasetime(self,a, mm, dl, time, period):
        rt, et=self.Area_Mission_time[a][mm]
        if rt<=time and et==-1:
            return True
        k=np.floor((time-dl)/(period*60))
        dltime=dl+(k+1)*(period*60)
        if rt<=dltime and et<=dltime:
            return True
        #print(f" {a} didn't release {time} {rt} {et}")
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
        #print(f"see cover area {areas}")
        ############ Update time 
        for m , dl in self.Mission_DL.items():  # Check the dl! 
            # if IFLOG:
            #     logfile.write(f"Drone {self.drone.id} JR  {m} {len(self.Mission_JR[m][0])}\n")
            if dl<self.time:
                if IFLOG:
                    logfile.write(f"Drone {self.drone.id} Miss Deadline  {m} DL {dl} at {self.time}\n")
                period=self.MissionDict[m[0]][0]
                k=np.floor((self.time-dl)/(period*60))
                ###############################
                NoUpDL=False
                for grid in self.Mission_Grid[m]:
                    #mm, offset, gid =grid
                    CurArea=[a for a in self.Grid_Area[grid] if self.CheckReleasetime(a,grid[:-1], dl, self.time, period)]
                    self.Grid_toCov[grid]=CurArea  # Update to Cover
                    self.Fix_toCov[grid]=copy.copy(CurArea)
                # if IFLOG:
                #     logfile.write(f"Drone {self.drone.id} fix cov {self.Fix_toCov}\n")
                #Here we need update something for release and  
                ############################
                #period=self.MissionDict[m[0]][0]
                weight=self.MissionDict[m[0]][1]
                #k=np.floor((self.time-dl)/(period*60))
                ndl=dl+(k+1)*(period*60)
                self.Mission_DL[m]=ndl  # 
                #print(f"see Update DL {m} {self.Mission_DL}")
                NumMiss=0
                for i in range(int(k)):
                    Cur_Mission=[a for a in self.Mission_Area.get(m) if self.CheckReleasetime(a,m, dl, dl+period*(i+1), period)]
                    NumMiss=NumMiss+len(Cur_Mission)
                Cur_Mission=[a for a in self.Mission_Area.get(m) if self.CheckReleasetime(a,m, dl, ndl, period)]
                #mcount=len(self.Mission_Area.get(m))
                Cur_miss=[a for a in self.Mission_JR[m][0] if self.CheckReleasetime(a,m, dl, ndl, period)]
                #jrcount=len(self.Mission_JR[m][0])
                jrcount=len(Cur_miss)
                if IFLOG:
                    logfile.write(f"Miss {m}  {jrcount} weight {weight}  at {self.time}\n")
                    
                    #logfile.write(f"Drone {self.drone.id} Miss  {m} {self.Mission_JR[m]}\n")
                Penalty=Penalty+(k*weight*self.sigma)*NumMiss+(jrcount*(weight*self.sigma))
                Reward=Reward-(k*weight*self.sigma)*NumMiss-(jrcount*(weight*self.sigma))   # Get the penalty! 
                for a in Cur_Mission:
                    self.TaskState[a][m]=(0,0) # update the jr and sr       
                self.Mission_JR[m]=defaultdict(list) 
                self.Mission_JR[m][0]=list(Cur_Mission)  # = =??? have to add list??s cluster update all is 0
                self.Mission_SR[m]=defaultdict(list)      # cluster update for sr=0
        ################## Upload Storage
        if CONNECT:
            if IFLOG:
                logfile.write(f"Drone {self.drone.id} Got connected at  {self.time} \n")

            for m , srdict in self.Mission_SR.items():  # clear the storage!!! 
                mission,offset=m
                for sv, ars in srdict.items():
                    for a in ars:
                        jr, sr=self.TaskState[a][m]
                        if sr>jr:
                            #print(f"see reward {Reward}")
                            try:
                                #print(f"fail {self.time} { self.Area_Mission_time[a][m]}")
                                #print(f"why fail? {a} {m} {self.TaskState[a][m]}  {self.Mission_JR[m]}")
                                self.Mission_JR[m][jr].remove(a)  
                                self.Mission_JR[m][sr].append(a)  
                                Reward=Reward+(sr-jr)   # Get the reward from storage! 
                            except:
                                print(f"see reward {Reward}")
                                print(f"something failed ")
                        self.TaskState[a][m]=(max(sr,jr),0) # update sr=0
                        #print(f" see if here changed? {m} {a} sr {sr} jr {jr}")
                self.Mission_SR[m]=defaultdict(list)   # clean storage! 
        if areas!=None: 
            #print(f"see cover areas {areas}")
            for a in areas:   # the areas the waypoint can cover!!!! 
                a=a[:-1]
                for m,st in self.TaskState[a].items():
                    mission,offset=m
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
                            Reward=Reward+(rwd-jr) # Add reward!!! 
                            self.TaskState[a][m]=(rwd,0) # update sr=0
                        except:
                            print(f"maybe area is not released {a}")
                    elif not CONNECT and rwd>max(sr,jr):   # if SR<JR is useless, right???  
                        try:
                            if sr>0:
                                self.Mission_SR[m][sr].remove(a)  # update state
                            self.Mission_SR[m][rwd].append(a)   # update cluster state 
                            SReward=SReward+(rwd-max(sr,jr))
                            self.TaskState[a][m]=(jr,rwd) # update sr=0
                        except:
                            print(f"maybe area is not released {a}")
        print(f"see reward {Reward},sreward {SReward}, penalty {Penalty}")
        return Reward, SReward, Penalty, NoUpDL

    def GetDataReward(self,wpc): # Get reward without updating state!!! 
        Reward=0; Covercount=0
        if self.wp!=wpc:
            distance=self.DisMatrix[self.wp][wpc]
        else:
            distance=0
        Ttime=distance/self.drone.speed+self.drone.loiter # maybe 1 s
        areas=self.Cover_map.get(wpc)
        ############ Update time 
        for a in areas:   # the areas the waypoint can cover!!!! 
            #print(f"see area {a[:-1]}")
            for m,st in self.TaskState[(a[:-1])].items():
                #print(m,st)
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
    
    def TrackState(self):
        print(f" DroneID: {self.drone.id}")
        print(f" waypoint: {self.wp}")
        print(f" time: {self.time}")
    
    def NeigWPC(self,areas,g): ### Now it is stupid! 1) Add checking reward method! 2) How to handle staying at the same locations? 
        mission, offset,gid=g
        WPCs=[]
        poheights=list(self.drone.WPCinfo.get(g[0]).keys())
        #print(f"see this time areas {len(areas)}  {areas}")
        for a in areas: 
            to_wp=[w for w in self.Area_WPC.get(a) if w[2] in (poheights)]
            maxh=max([ww[2] for ww in to_wp])
            to_wp=[w for w in to_wp if w[2]==maxh]
            WPCs=WPCs+to_wp
        WPCs=list(set(WPCs))
        WPCs=[w for w in WPCs if not (self.CheckReturn(w,((g[0],g[1]),self.Mission_DL[g[0],g[1]]))) ]
        #print(f"See WPCs {WPCs}")
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
        poheights=list(self.drone.WPCinfo.get(g[0]).keys())
        for a in areas: 
            to_wp=[w for w in self.Area_WPC.get((a,mission)) if w[2] in (poheights)]
            # maxh=max([ww[2] for ww in to_wp])
            # to_wp=[w for w in to_wp if w[2]==maxh]
            WPCs=WPCs+to_wp
        WPCs=list(set(WPCs))
        print([self.CheckReturn(w,((g[0],g[1]),self.Mission_DL[g[0],g[1]])) for w in WPCs])
        WPCs=[w for w in WPCs if not (self.CheckReturn(w,((g[0],g[1]),self.Mission_DL[g[0],g[1]]))) ]
        print(f"See WPCs {WPCs}")
        if len(WPCs)==0:
            nwpc=self.ShortUpload.get(self.wp)[1]
            print(f"No WPCs is valid--> Return back!")
            return nwpc, True
        else:
            Covlist=[self.GetDataReward(w) for w in WPCs]
            maxrwd=max([x[2]/x[1] for x in Covlist])
            tmp=[i for i in list(range(len(Covlist))) if Covlist[i][2]/Covlist[i][1]==maxrwd]
            print(f"Covlist {maxrwd} {[WPCs[i] for i in tmp]}\n")
            if IFLOG:
                logfile.write(f"Covlist {maxrwd} {[WPCs[i] for i in tmp]}\n")
            x,y,z=max(Covlist, key=lambda x: x[2]/x[1])
            #print(f"which cover more? {(x,y,z)}")
            #Diss=[self.Distance(self.wp,w) for w in WPCs] #Here the distance can be compute less????
            adex=Covlist.index((x,y,z))
            nwpc=WPCs[adex]
            return nwpc, False

    def BestRewardWPC(self,areas,g): ### Now it is stupid! 1) Add checking reward method! 2) How to handle staying at the same locations? 
        print(f" Enter here")
        mission, offset,gid=g
        
        WPCs=[]
        poheights=list(self.drone.WPCinfo.get(g[0]).keys())
        for a in areas: 
            to_wp=[w for w in self.Area_WPC.get((a,mission)) if w[2] in (poheights)]
            # maxh=max([ww[2] for ww in to_wp])
            # to_wp=[w for w in to_wp if w[2]==maxh]
            WPCs=WPCs+to_wp
        WPCs=list(set(WPCs))
        print([self.CheckReturn(w,((g[0],g[1]),self.Mission_DL[g[0],g[1]])) for w in WPCs])
        WPCs=[w for w in WPCs if not (self.CheckReturn(w,((g[0],g[1]),self.Mission_DL[g[0],g[1]]))) ]
        print(f"See WPCs {WPCs}")
        if len(WPCs)==0:
            nwpc=self.ShortUpload.get(self.wp)[1]
            print(f"No WPCs is valid--> Return back!")
            return nwpc, True
        else:
            Covlist=[self.GetDataReward(w) for w in WPCs]
            x,y,z=max(Covlist, key=lambda x: x[0]/x[1])
            if x==0:
                nwpc=self.ShortUpload.get(self.wp)[1]
                return nwpc,True
            #print(f"which cover more? {(x,y,z)}")
            #Diss=[self.Distance(self.wp,w) for w in WPCs] #Here the distance can be compute less????
            adex=Covlist.index((x,y,z))
            nwpc=WPCs[adex]
            return nwpc, False




    def CheckReturn(self, wpc,mission):  # check it this waypoint is valid 
        #print(f"why??? {self.Connection[wpc]}")
        #if not self.Connection[self.wp]:
        Delay=0
        if self.wp!=wpc:
            Delay=(self.DisMatrix[self.wp].get(wpc))/self.drone.speed+self.drone.loiter
        time, _=self.ShortUpload.get(wpc)
        Delay=Delay+time
        #print(f"see deadline mission {mission}")
        if (mission[1] -self.time)<Delay:      # Maybe storage is empty, but go to  waypoint out of connection
            #print(f" Return! fly is late {mission[0]} {mission[1] -self.time} < {Delay}")
            #tt, nwpc=self.ShortUpload.get(self.wp) # self.wp must be not connected, as we have 
            #print(f"see return time {tt}") 
            return True
        # else:
        #     print(f"not late: {mission[0]} {mission[1] -self.time} > {Delay} {time} ?  ")
        for m, tt in self.Mission_SR.items():
            #print(f"see should {tt} {m} {self.Mission_DL[m]} {self.time} {Delay} ")
            if len(tt)>0 and (self.Mission_DL[m] -self.time)<Delay: # If drone has storage, then check 
                #print(f"see should {m} {self.Mission_DL[m]} {self.time} {Delay} ")
                #print(f"Need to return {Delay} < {self.Mission_DL[m]-self.time}")
                #tt, nwpc=self.ShortUpload.get(self.wp) # self.wp must be not connected, as we have 
                #print(f"see this time {tt} ") 
                return True
        #print(f"Exist? {self.time} {Delay}")
        return False
    def Cluster_Scheduling(self,DecomposeSize, Res): 
        #self.DrawWPCs()
        self.waypointSeq.append(self.wp)
        ########Define a NOWPCGrid 
        NOWPCGrid=defaultdict(dict)
        Reward=0; P=0
        self.initialTask()
        self.ClusterTask()
        st=time.time()
        EndTime=1200  # Unit is second
        sr=DecomposeSize/Res
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
                # Fulfill the tasks in the grid with earliest deadlines 
                CurGrids=copy.copy(Gridset[count])
                CurGrids=[g for g in CurGrids if len(self.Grid_toCov[g])>0 ]
                while len(CurGrids)>0 and NoUpDL and self.time<=EndTime:
                    ##### Find the grid closest to self.wp
                    Diss=[self.Distance(self.wp, (g[-1][0]*sr,g[-1][1]*sr,0)) for g in CurGrids]
                    adex=Diss.index(min(Diss)) 
                    near_g=CurGrids[adex] 
                    NoWPCs=False
                    #print(f"see time {self.time}")
                    #CurDL=self.Mission_DL.get(near_g[:-1])
                    #print(f" Enter  {near_g}  ") 
                    if IFLOG:
                        logfile.write(f"Drone {self.drone.id} Enter task {near_g} at {self.time}\n")
                    while len(self.Grid_toCov[near_g])>0 and (NoUpDL and self.time<=EndTime):
                        areas=self.Grid_toCov[near_g]
                        nwpc, NoWPCs=self.BestCovNeigWPC(areas,near_g) # Here near_g is 
                        if NoWPCs:
                            if IFLOG:
                                logfile.write(f"Drone {self.drone.id} No WPC at {near_g} \n")
                            break # Did not add waypoint! 
                        self.waypointSeq.append(nwpc)
                        reward,SReward,Penalty,NoUpDL=self.ClusterStateTrans(nwpc)
                        #self.DrawWPsequence()
                        Reward=Reward+reward
                        P=P+Penalty
                    if NoUpDL and self.time<=EndTime: #  Remove the area. 
                        CurGrids.remove(near_g)
                        CurGrids=[g for g in CurGrids if len(self.Grid_toCov[g])>0 ]
                    if not NoUpDL:
                        print(f"Get out of one deadline {count}, Or Update {NoUpDL} Or timeout {self.time<=EndTime}")
                count=count+1
                if count==len(Gridset) and NoUpDL:# Now, there is no NoUPLOAD! 
                    print(f"No grid to choose, upload data")
                    SRlength=max([len(k) for k in self.Mission_SR.values()])
                    if SRlength==0: # Means drone storage is empty.
                        count=0
                        while NoUpDL and count<len(Gridset): #Enter improve phase 
                            #areas=[a for a in self.Grid_Area[grid] if self.CheckReleasetime(a,grid[:-1], dl, self.time, period)]
                            #self.Grid_toCov[grid]=CurArea  # Update to Cover
                            CurGrids=copy.copy(Gridset[count])
                            near_g=CurGrids[0] 
                            if IFLOG:
                                logfile.write(f"Drone {self.drone.id} Improve {near_g}\n")
                                #logfile.write(f" fix cov{self.Fix_toCov} \n")
                            nwpc, NoWPCs=self.BestRewardWPC(self.Fix_toCov[near_g],near_g) # Here near_g is 
                            if NoWPCs:
                                count=count+1
                                break
                            self.waypointSeq.append(nwpc)
                            reward,SReward,Penalty,NoUpDL=self.ClusterStateTrans(nwpc)
                            if IFLOG:
                                logfile.write(f"Drone {self.drone.id} fly by {self.time} with {reward} {SReward}???????????????\n")
                            Reward=Reward+reward
                            P=P+Penalty
                        # if SRlength>0:
                        #     minDL=min([self.Mission_DL[m] for m in list(self.Mission_SR.keys()) if len(self.Mission_SR[m])>0])
                        # else:
                        #     minDL=0
                        # logfile.write(f" Storage is {minDL} {self.Mission_SR} {self.time}\n")
                    nwpc=self.ShortUpload.get(self.wp)[1]
                    self.waypointSeq.append(nwpc)
                    reward,SReward,Penalty,NoUpDL=self.ClusterStateTrans(nwpc)
                    Reward=Reward+reward
                    P=P+Penalty
                    if IFLOG:
                        logfile.write(f"Drone {self.drone.id} Upload data at {self.time}\n")
                    # if NoUpDL:
                    count=0               
            #reward, sreward, penal=self.MissionEDF_Decision()
            #reward, sreward, penal=self.Single_Step_Valid_Decision() #Check Single-step Checking
            #reward, sreward, penal=self.More_Step_Valid_Decision() #Check Single-step Checking
            #reward, sreward, penal,UpdateGrid=self.Cluster_Valid_Decision(200,10,UpdateGrid) #Check Single-step Checking
        print(f"Planning time {time.time()-st}")    
        print(f" drone {self.drone.id} waypoints {Reward}, {P}, {len(self.waypointSeq)} {self.waypointSeq}")
        
        return self.waypointSeq
    def WCPCosArea(self, areas,g):   # Too simple, To get the waypoint cover the closet areas 
        Diss=[self.Distance(self.wp, (a[0],a[1],0)) for a in areas] #Here the distance can be compute less????
        adex=Diss.index(min(Diss))
        near_a=areas[adex]
        to_wp=self.Area_WPC.get(near_a)
        poheights=list(self.drone.WPCinfo.get(g[0]).keys())
        wps=[w for w in to_wp if w[2] in poheights]   # why did not match????? 
        wps.sort(key=lambda s: s[2], reverse=True)
        Return=self.CheckReturn(wps[0],((g[0],g[1]),self.Mission_DL[g[0],g[1]]))
        #nwpc=wps[0]
        if Return:
            nwpc=self.ShortUpload.get(self.wp)[1]
        else:
            nwpc=wps[0]
        return nwpc

    def Scheduling(self): 
        #self.DrawWPCs()
        self.waypointSeq.append(self.wp)
        Reward=0; P=0
        print(f" length", len(self.WPCHeight))   # WPCs' key is the height: waypoints; Cover_map's key is waypoint: areas
        self.initialTask()
        self.ClusterTask()
        st=time.time()
        while self.time<=1200:
            #reward, sreward, penal=self.MissionEDF_Decision()
            #reward, sreward, penal=self.Single_Step_Valid_Decision() #Check Single-step Checking
            #reward, sreward, penal=self.More_Step_Valid_Decision() #Check Single-step Checking
            reward, sreward, penal,UpdateGrid=self.Cluster_Valid_Decision(200,10,UpdateGrid) #Check Single-step Checking
            Reward=Reward+reward
            P=P+penal
        print(f"Planning time {time.time()-st}")    
        print(f" drone {self.drone.id} waypoints {Reward}, {P}, {len(self.waypointSeq)} {self.waypointSeq}")
        self.DrawWPsequence()

    
if __name__ == "__main__":
    Missions=defaultdict(dict)  #  Missions: id, period, priority, 
    Missions['BM']=[ 10, 1]  #Other area 
    Missions['FI']=[ 5, 1]  #track the fire 
    Missions['FT']=[ 3, 3]  # track the fire perimeter and the risky area (arrival within 10 min)
    #Missions['FL']=[ 3, 2]  # FL is tracking the fire perimeter.
    #Missions['FD']=[ 2, 3]
    ################ Get tasks.
    dir='/home/fangqiliu/eclipse-workspace_Python/Drone_path/CoveragePathPlanning-master/farsite'
    foldername='FQ_sim'
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
    #CreateDyRxfire(Bardata, Bursite,'FQ_burn',dir, [2])
    EFA,EFAdict,Primdict =GetFirSim(Bardata, Bursite,  foldername, 'FQ_burn', dir, Bursite, Res=Res,time=40)                  
    TM=TaskManager(Missions)
    tasks=TM.DeclareGridEFA(EFA,init=0)
    print(f"check task {tasks}")
    DrawTask(tasks,EFA) 
    ########################################
    sensorfile='Data/sensor_info.csv'
    PPMfile='Data/PPM_table.csv'
    DroneNum=5
    speeds=[5,5,5,5,3,3]
    loiter=[1,2,1,1,1]
    ranges=[300,200,500,300,300]
    GCloc=(0,500)
    sensgrp=[['ZENMUSE_XT2_t','ZENMUSE_XT2_r'],['DJI_Air2S'],['ZENMUSE_XT2_t','ZENMUSE_XT2_r'],['DJI_Air2S'],['ZENMUSE_XT2_t','ZENMUSE_XT2_r']]
    Drones=LoadDrones(sensorfile,PPMfile,DroneNum, speeds, sensgrp, Res,loiter,ranges)
    # for d in Drones:
    #     print(f"for drone {d.id} ")
    #     d.GenWPCandidate(EFA,Res,Missions)
    ###################### Generate flight plan! 
    AreaPara=Decomposition(DecomposeSize,Res,tasks) # Need put tasks there!!!
    Drones,Bidders,Grids=Auction(AreaPara, Drones, Res,Missions,3, EFA,GCloc)
    print(f"see Drones cost {[Bidders[i].coverarea for i in range(len(Drones))]}")
    TrackDraw(Drones, EFA)
    #################################
    init=1; end=40
    for drone in Drones[4:5]:
        planner=FlightPlanner(drone,init=init, planTime=10, iniloc=(0,50,0), GCloc=(0,50,0),Missions=Missions,Res=Res, DecomposeSize=DecomposeSize) # Here, GCloc and iniloc divided by Res!
        planner.GenWaypoint()
        WPsequence=planner.Cluster_Scheduling(200,10)
        planner.DrawWPsequence()
        #plt.show()
        
    #logfile.close()
    

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









