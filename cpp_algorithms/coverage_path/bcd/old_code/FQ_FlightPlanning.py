
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from bcd_helper import imshow, imshow_scatter
import math
from collections import defaultdict
import random
from FQ_TaskAllocation import Decomposition, Auction, TrackDraw
from FQ_GenTask import GenTasks
from FQ_Drones_Info import Sensor, Drone, ReadSen_PPM,LoadDrones
import time

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
    def __init__(self, Drone, init=0, planTime=10, iniloc=[], GCloc=[], endloc=[], Missions={}, Res=10):
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
        self.TaskState=defaultdict(list)# Deadline, Reward, stored reward, Acum. Penalty
        self.WPCs=defaultdict(list)
        self.Cover_map=defaultdict(list)
        self.Area_WPC=defaultdict(list)
        self.Grid_Task=defaultdict(list)
        self.Connection={} #should be waypoint:0/1 yes or no has network!!!
        self.sigma=100
        self.waypointSeq=[]
        self.missioncluster=defaultdict(list)
        #Then I need to the the location of the GC!!!
    def GenWaypoint(self):
        for grid in self.tasks:#Get the minimum and maximum x
            areas=grid[3]
            tak=grid[0]
            for a in areas:
                self.Grid_Task[a[0], a[1]].append((tak,a[2]))
            xal=[x[0] for x in areas]
            yal=[x[1] for x in areas]
            heights=self.drone.WPCinfo.get(tak)
            ############ To delete the redundant heights with the same coverage area. 
            tmph=[[h, heights[h][0]] for h in heights]
            tmph=sorted(tmph, key=lambda l:l[0], reverse=False) 
            tst=[];delset=[]
            for t in tmph:
                if t[1] not in tst:
                    tst.append(t[1])
                else:
                    delset.append(t)
            for d in delset:
                tmph.remove(d)
            for wpc in tmph:
                fov=wpc[1]/(self.Res)
                x=min(xal)
                while x<=max(xal):
                    y=min(yal)
                    while y<=max(yal):
                        self.WPCs[wpc[0]].append((x+fov/2,y+fov/2,wpc[0]))
                        conn=(self.Distance((x+fov/2,y+fov/2,wpc[0]), self.GCloc)<=self.drone.range)
                        self.Connection[x+fov/2,y+fov/2,wpc[0]]=conn
                        ########################### For get the coverage area
                        if self.Cover_map.get((x+fov/2,y+fov/2,wpc[0]))==None:
                               #print(f"see get duplicate {x+fov/2,y+fov/2,wpc[0]} {self.Cover_map.get((x+fov/2,y+fov/2,wpc[0]))}")
                            cx=int(x)
                            while cx<x+fov:
                               cy=int(y) 
                               while cy<y+fov:
                                   self.Cover_map[x+fov/2,y+fov/2,wpc[0]].append((cx,cy))
                                   self.Area_WPC[cx,cy].append((x+fov/2,y+fov/2,wpc[0]))
                                   cy=cy+1
                               cx=cx+1
                        ########################
                        y=y+fov
                    x=x+fov
    def initialTask(self):     #Here we only consider to handle started task!
        for g in self.Grid_Task:
            for t in self.Grid_Task[g]: 
                DL= max(t[1],self.time)+(self.MissionDict[t[0]][0])*60
                self.TaskState[g].append((t[0],DL,0,0,0))# Deadline job reward, stored reward, accumulative penalty. 

        
    def StateTrans(self,wpc):
        distance=np.sqrt(((self.wp[0]-wpc[0])*self.Res)**2+((self.wp[1]-wpc[1])*self.Res)**2+(self.wp[2]-wpc[2])**2)
        time=distance/self.drone.speed+self.drone.loiter # maybe 1 s
        self.time=self.time+time
        self.wp=wpc
        CONNECT=self.Connection[wpc]
        Reward=0
        areas=self.Cover_map.get(wpc)
        for a in self.TaskState:
            updlist=[]
            for task in self.TaskState.get(a):
                mission,dl,jr, sr,cp =task
                if self.time>dl:   # must be 0, right!!!
                    period=self.MissionDict[mission][0]
                    weight=self.MissionDict[mission][1]
                    k=np.floor((self.time-dl)/(period*60))
                    cp=cp+k*weight*self.sigma
                    if jr>0: 
                        Reward=Reward+jr
                    else: 
                        cp=cp+weight*self.sigma
                    dl=dl+(k+1)*(period*60)
                    jr=0;sr=0
                if a in areas: # Reward may >0
                    poheights=list(self.drone.WPCinfo.get(task[0]).keys())
                    z=wpc[2]
                    if z>max(poheights):
                        reward=0
                    else:
                        if z<min(poheights):
                            z=min(poheights)
                        elif z not in poheights:
                            z=min([k for k in poheights if k>z])
                        reward=self.drone.WPCinfo.get(task[0]).get(z)[1] # get the wpcinfo, get the reward. 
                    if CONNECT:
                        jr=max([jr, reward,sr]); sr=0
                    else:
                        sr=max(sr, reward)
                elif CONNECT and sr>0:
                    jr=max(jr,sr);sr=0
                updlist.append((mission,dl,jr, sr,cp))
                if self.time==dl:
                    Reward=Reward+jr
            self.TaskState[a]=updlist
        #print(f"wpc {wpc} time {self.time/60}")
        return Reward
    def CheckUpdateTask(self, time): # Add/remove task which start after initial time. 
        pass
    def FastStateTrans(self,wpc):
        distance=np.sqrt(((self.wp[0]-wpc[0])*self.Res)**2+((self.wp[1]-wpc[1])*self.Res)**2+(self.wp[2]-wpc[2])**2)
        time=distance/self.drone.speed+self.drone.loiter # maybe 1 s
        self.time=self.time+time
        self.wp=wpc
        CONNECT=self.Connection[wpc]
        Reward=0
        areas=self.Cover_map.get(wpc)
        for a in self.TaskState:
            updlist=[]
            for task in self.TaskState.get(a):
                mission,dl,jr, sr,cp =task
                if self.time>dl:   # must be 0, right!!!
                    period=self.MissionDict[mission][0]
                    weight=self.MissionDict[mission][1]
                    k=np.floor((self.time-dl)/(period*60))
                    cp=cp+k*weight*self.sigma
                    if jr>0: 
                        Reward=Reward+jr
                    else: 
                        cp=cp+weight*self.sigma
                    dl=dl+(k+1)*(period*60)
                    jr=0;sr=0
                if a in areas: # Reward may >0
                    poheights=list(self.drone.WPCinfo.get(task[0]).keys())
                    z=wpc[2]
                    if z>max(poheights):
                        reward=0
                    else:
                        if z<min(poheights):
                            z=min(poheights)
                        elif z not in poheights:
                            z=min([k for k in poheights if k>z])
                        reward=self.drone.WPCinfo.get(task[0]).get(z)[1] # get the wpcinfo, get the reward. 
                    if CONNECT:
                        jr=max([jr, reward,sr]); sr=0
                    else:
                        sr=max(sr, reward)
                elif CONNECT and sr>0:
                    jr=max(jr,sr);sr=0
                updlist.append((mission,dl,jr, sr,cp))
                if self.time==dl:
                    Reward=Reward+jr
            self.TaskState[a]=updlist
        return Reward
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    def Distance(self,w1, w2):
        dis=np.sqrt(((w1[0]-w2[0])*self.Res)**2+((w1[1]-w2[1])*self.Res)**2+((w1[2]-w2[2]))**2)
        return dis
        
    def DrawWPCs(self):
        fig = plt.figure(figsize=(10, 10))
        ax = fig.add_subplot(projection='3d')
        for h in self.WPCs:
            wpcs=self.WPCs[h]
            conwpc=[w for w in wpcs if self.Connection.get(w)==True]
            diswpc=[w for w in wpcs if self.Connection.get(w)==False]
            ax.scatter([self.GCloc[0]*self.Res],[self.GCloc[1]*self.Res],[self.GCloc[2]],color='green',s=200,marker='*')
            ax.scatter([w[0]*self.Res for w in conwpc ], [w[1]*self.Res for w in conwpc ], [w[2] for w in conwpc],color='blue')
            ax.scatter([w[0]*self.Res for w in diswpc ], [w[1]*self.Res for w in diswpc], [w[2]for w in diswpc],color='orange')
                #ax.scatter([k[0]*self.Res for k in ww], [k[1]*self.Res for k in ww], [k[2] for k in ww],color='blue')
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_zlabel('Z (m)')
        plt.show()
        
    def Scheduling(self): 
        #self.DrawWPCs()
        print(f" length", len(self.WPCs))   # WPCs' key is the height: waypoints; Cover_map's key is waypoint: areas
        # for w in self.WPCs:
        #     print(w, self.WPCs[w]) # Gird_Task is the area: tasks
        # for c in self.Cover_map:
        #     print(c, self.Cover_map[c])
        self.initialTask()
        while self.time<=150:
            self.EDF_Decision()
        print(f" drone {self.drone.id} has waypoint sequence {self.waypointSeq}")
        #print(f"why {self.WPCs}")
        # for height in self.WPCs:
        #     wpc=self.WPCs[height]
        #     print(len(wpc))
        #     for w in wpc:
        #         #print(f"w {w}")
        #         print(w)
        #self.StateTrans(w)
    def TrackState(self):
        print(f" DroneID: {self.drone.id}")
        print(f" waypoint: {self.wp}")
        print(f" time: {self.time}")
    def EDF_Decision(self): #select waypoint with earliest deadline
        State_Dict=defaultdict(list)
        print(f"check time {len(self.TaskState.items())}")
        for i in self.TaskState.items():   # every time go over this! too slow! 
            for ts in i[1]:
                State_Dict[ts].append(i[0])
        # Do we need check this every time?????? 
        print(f"see State {ts} {State_Dict[ts]}")  
        ts=list(State_Dict.keys())
        ts.sort(key=lambda s: s[1])        
        areas=State_Dict.get(ts[0])
        Diss=[self.Distance(self.wp, (a[0],a[1],0)) for a in areas]
        near_a=[a for a in areas if self.Distance(self.wp, (a[0],a[1],0))==min(Diss)][0]
        #print(f"Near a", near_a)
        to_wp=self.Area_WPC.get(near_a)
        #print(f"cover {to_wp}")
        to_wp.sort(key=lambda s: s[2], reverse=True)
        self.waypointSeq.append(to_wp[0])
        st=time.time()
        #print(f"see time")
        self.StateTrans(to_wp[0])
        
        print(f"see time {time.time()-st} ")
        #self.TrackState()
if __name__ == "__main__":
    Missions=defaultdict(dict)  #  Missions: id, period, priority, 
    Missions['BM']=[ 10, 1]  #Other area 
    Missions['FI']=[ 5, 1]  #track the fire 
    Missions['FT']=[ 3, 3]  # track the fire perimeter and the risky area (arrival within 10 min)
    Missions['FL']=[ 3, 2]  # FL is tracking the fire perimeter.
    Missions['FD']=[ 2, 3]
    ################ Get tasks.
    dir='/home/fangqiliu/eclipse-workspace_Python/Drone_path/CoveragePathPlanning-master/farsite/'
    foldername='FQ_burn'
    Bursite=(702460.0,4309700.0,703540.0,4310820.0 )
    init=120; end=150; Res=10
    Task_mission,BSshp=GenTasks(init,end,Bursite,dir,foldername,Missions, Res=Res)
    ########################## Do decomposition~ 
    DecomposeSize=200
    AreaPara=Decomposition(DecomposeSize,Res,Task_mission) # Need put tasks there!!!
    ########################## Get drones
    sensorfile='Data/sensor_info.csv'
    PPMfile='Data/PPM_table.csv'
    DroneNum=2
    speeds=[5,5,5,5,5,5]
    loiter=[1,2,1,1,1]
    sensgrp=[['ZENMUSE_XT2_t','ZENMUSE_XT2_r'],['DJI_Air2S'],['ZENMUSE_XT2_t','ZENMUSE_XT2_r']]
    Drones=LoadDrones(sensorfile,PPMfile,DroneNum, speeds, sensgrp, Res,loiter)
    ##################################################### Task Allocation 
    Drones,Bidders=Auction(AreaPara, Drones, Res,Missions,Task_mission,1)
    p=TrackDraw(Drones, BSshp, AreaPara)
    for drone in Drones:
        planner=FlightPlanner(drone,init=init, planTime=10, iniloc=(50,0,0), GCloc=(50,0,0),Missions=Missions,Res=Res) # Here, GCloc and iniloc divided by Res!
        planner.GenWaypoint()
        planner.Scheduling()
        #TrackDraw(Drones, BSshp, AreaPara)
        #imshow(p)
        #plt.show()
    
    
    
    
    
    
    
    
    
    
    
    
    












