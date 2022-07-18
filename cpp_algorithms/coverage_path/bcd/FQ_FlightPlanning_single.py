
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
from reportlab.lib._rl_accel import Penalty

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
        self.Cover_map=defaultdict(list)
        self.Area_WPC=defaultdict(list)
        self.Grid_Task=defaultdict(list)
        self.Connection={} #should be waypoint:0/1 yes or no has network!!!
        self.Con_wpc=[]
        self.sigma=10
        self.waypointSeq=[]
        self.Mission_Area=defaultdict(list)
        self.Mission_JR=defaultdict(dict)
        self.Mission_SR=defaultdict(dict)
        self.Mission_DL={}
        self.DisMatrix=defaultdict(dict)
        self.ShortUpload={}
        self.Area_grid={} # for mapping area to grid! {area: (grid, mission)}
        self.Grid_tocov=defaultdict(list) # for a grid, some area need be covered
        #Then I need to the the location of the GC!!!
    def GenWaypoint(self):
        for grid in self.tasks:#Get the minimum and maximum x
            
            areas=grid[3]
            tak=grid[0]
            #print(f"I want to see what is grid {grid}")
            for a in areas:
                self.Grid_Task[a[0], a[1]].append((tak,a[2]))
                self.Area_grid[a[0], a[1]]=(grid[1],tak)   #################### FQFQ update
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
                        self.WPCHeight[wpc[0]].append((x+fov/2,y+fov/2,wpc[0]))
                        self.WPCs.append((x+fov/2,y+fov/2,wpc[0]))
                        conn=(self.Distance((x+fov/2,y+fov/2,wpc[0]), self.GCloc)<=self.drone.range)
                        self.Connection[x+fov/2,y+fov/2,wpc[0]]=conn
                        if conn==True:
                            self.Con_wpc.append((x+fov/2,y+fov/2,wpc[0]))
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
        st=time.time()
        for i in range(len(self.WPCs)):
            for j in range(i+1,len(self.WPCs)):
                self.DisMatrix[self.WPCs[i]][self.WPCs[j]]=self.Distance(self.WPCs[i], self.WPCs[j])
                self.DisMatrix[self.WPCs[j]][self.WPCs[i]]=self.DisMatrix[self.WPCs[i]][self.WPCs[j]]
        print(f"time to compute distance {time.time()-st}")
        for w in list(set(self.WPCs)-set(self.Con_wpc)):
            Dis=[self.DisMatrix[w][ww] for ww in self.Con_wpc]
            wm=Dis.index(min(Dis))
            #print(f"why distance {min(Dis)}")
            Ttime=min(Dis)/self.drone.speed+self.drone.loiter # maybe 1 s
            #print(f"why {self.drone.speed}, {Ttime}")
            self.ShortUpload[w]=(Ttime,self.Con_wpc[wm])
        for w in self.Con_wpc:
            self.ShortUpload[w]=(self.drone.loiter,w) # if is connected, it is itself. 
        print(f"time to compute short upload {time.time()-st}")
        
                 
    def initialTask(self):     #Here we only consider to handle started task!
        for g in self.Grid_Task:
            for t in self.Grid_Task[g]: 
                offset= (max(t[1],self.time)-self.time)%((self.MissionDict[t[0]][0])*60)
                self.TaskState[g][t[0],offset]=(0,0)# Deadline job reward, stored reward. 
    def ClusterTask(self):# cluster tasks based on their mission
        tmp_jr=defaultdict(list)
        tmp_sr=defaultdict(list)
        for a in self.TaskState:
            
            for m,st in self.TaskState[a].items():
                mission,offset=m
                jr,sr=st
                self.Mission_Area[mission,offset].append(a)
                tmp_jr[mission,offset].append((jr,a))
                if sr>0:
                    tmp_sr[mission,offset].append((sr,a))
        for key in tmp_jr:
            dict_jr=defaultdict(list)
            for p in tmp_jr[key]:
                dict_jr[p[0]].append(p[1])
            self.Mission_JR[key]=dict_jr
            dict_jr=defaultdict(list)
            for p in tmp_sr[key]:
                dict_jr[p[0]].append(p[1])                
            self.Mission_SR[key]=dict_jr
            self.Mission_DL[key]=offset+((self.MissionDict[key[0]][0])*60)
        #print(f"whyyyy? check (66, 50) {self.Mission_JR['FT', 0].items()} ")

    def CheckUpdateTask(self, time): # Add/remove task which start after initial time. 
        pass
    def FastStateTrans(self,wpc):
        Reward=0; SReward=0; Penalty=0
        DLUpdate=False
        if self.wp!=wpc:
            distance=self.DisMatrix[self.wp][wpc]
        else:
            distance=0
        Ttime=distance/self.drone.speed+self.drone.loiter # maybe 1 s
        self.time=self.time+Ttime
        self.wp=wpc
        CONNECT=self.Connection[wpc]
        areas=self.Cover_map.get(wpc)
        ############ Update time 
        for m , dl in self.Mission_DL.items():  # Check the dl! 
            if dl<self.time:
                period=self.MissionDict[m[0]][0]
                weight=self.MissionDict[m[0]][1]
                k=np.floor((self.time-dl)/(period*60))
                dl=dl+(k+1)*(period*60)
                self.Mission_DL[m]=dl  # 
                DLUpdate=True
                mcount=len(self.Mission_Area.get(m))
                jrcount=len(self.Mission_JR[m][0])
                Penalty=Penalty+(k*weight*self.sigma)*mcount+(jrcount*(weight*self.sigma))
                Reward=Reward-(k*weight*self.sigma)*mcount-(jrcount*(weight*self.sigma))   # Get the penalty! 
                for a in self.Mission_Area.get(m):
                    self.TaskState[a][m]=(0,0) # update the jr and sr       
                self.Mission_JR[m]=defaultdict(list) 
                self.Mission_JR[m][0]=list(self.Mission_Area.get(m))  # = =??? have to add list??s cluster update all is 0
                self.Mission_SR[m]=defaultdict(list)      # cluster update for sr=0
        ################## Upload Storage
        if CONNECT:
            for m , srdict in self.Mission_SR.items():  # clear the storage!!! 
                mission,offset=m
                for sv, ars in srdict.items():
                    for a in ars:
                        jr, sr=self.TaskState[a][m]
                        if sr>jr:
                            Reward=Reward+(sr-jr)   # Get the reward from storage! 
                            self.Mission_JR[m][jr].remove(a)  
                            self.Mission_JR[m][sr].append(a)  
                        self.TaskState[a][m]=(max(sr,jr),0) # update sr=0
                        #print(f" see if here changed? {m} {a} sr {sr} jr {jr}")
                self.Mission_SR[m]=defaultdict(list)   # clean storage! 
        for a in areas:   # the areas the waypoint can cover!!!! 
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
                #print(f"{a} Reward  {rwd} jr {jr} sr {sr} {CONNECT} ")
                #######################################    
                if CONNECT and rwd>jr:
                    Reward=Reward+(rwd-jr) # Add reward!!! 
                    self.TaskState[a][m]=(rwd,0) # update sr=0
                    self.Mission_JR[m][jr].remove(a)  
                    self.Mission_JR[m][rwd].append(a)  
                elif not CONNECT and rwd>max(sr,jr):   # if SR<JR is useless, right???  
                    SReward=SReward+(rwd-max(sr,jr))
                    self.TaskState[a][m]=(jr,rwd) # update sr=0
                    if sr>0:
                        self.Mission_SR[m][sr].remove(a)  # update state
                    self.Mission_SR[m][rwd].append(a)   # update cluster state 
        print(f"see reward {Reward},sreward {SReward}, penalty {Penalty}")
        return Reward, SReward, Penalty, DLUpdate

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
            for m,st in self.TaskState[a].items():
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
        print(f"see reward {Reward}")# penalty {Penalty}")
        return Reward, Ttime, Covercount

    def GetNeighbors(self,wpc): ### Get the possible waypoint neighbors to select
        # Mission based?
        # Depending on location! find surrounding waypoints within specific distance
        # 3) Or cell-based neighbors???
        
        
        
        
        pass
        

    def Distance(self,w1, w2):
        dis=np.sqrt(((w1[0]-w2[0])*self.Res)**2+((w1[1]-w2[1])*self.Res)**2+((w1[2]-w2[2]))**2)
        return dis
        
        
    def DrawWPCs(self):
        fig = plt.figure(figsize=(10, 10))
        ax = fig.add_subplot(projection='3d')
        for h in self.WPCHeight:
            wpcs=self.WPCHeight[h]
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
    def DrawWPsequence(self):
        fig=plt.figure()
        ax=plt.axes(projection='3d')
        ax.plot3D([i[0] for i in self.waypointSeq], [i[1] for i in self.waypointSeq],[i[2] for i in self.waypointSeq])
        ax.scatter([i[0] for i in self.waypointSeq], [i[1] for i in self.waypointSeq],[i[2] for i in self.waypointSeq])
        conw=[w for w in self.waypointSeq ]
        plt.show()
    def Scheduling(self): 
        #self.DrawWPCs()
        self.waypointSeq.append(self.wp)
        Reward=0; P=0
        print(f" length", len(self.WPCHeight))   # WPCs' key is the height: waypoints; Cover_map's key is waypoint: areas
        self.initialTask()
        self.ClusterTask()
        st=time.time()
        
        while self.time<=1800:
            #reward, sreward, penal=self.MissionEDF_Decision()
            #reward, sreward, penal=self.Single_Step_Valid_Decision() #Check Single-step Checking
            #reward, sreward, penal=self.More_Step_Valid_Decision() #Check Single-step Checking
            reward, sreward, penal,UpdateGrid=self.Cluster_Valid_Decision(200,10,UpdateGrid) #Check Single-step Checking
            Reward=Reward+reward
            P=P+penal
        print(f"Planning time {time.time()-st}")    
        print(f" drone {self.drone.id} waypoints {Reward}, {P}, {len(self.waypointSeq)} {self.waypointSeq}")
        self.DrawWPsequence()

    def TrackState(self):
        print(f" DroneID: {self.drone.id}")
        print(f" waypoint: {self.wp}")
        print(f" time: {self.time}")
    def MissionEDF_Decision(self):
        dlist=list(self.Mission_DL.items())  # Check the dl! 
        dlist.sort(key=lambda s: s[1])
        for m in dlist:
            if len(self.Mission_JR[m[0]][0])>0:
                areas=self.Mission_JR[m[0]][0]
                for key, aa in self.Mission_SR[m[0]].items():
                    areas=list(set(areas)-set(aa))
                if len(areas)>0:
                    break
        Diss=[self.Distance(self.wp, (a[0],a[1],0)) for a in areas]
        adex=Diss.index(min(Diss))
        near_a=areas[adex]
        to_wp=self.Area_WPC.get(near_a)
        poheights=list(self.drone.WPCinfo.get(m[0][0]).keys())
        wps=[w for w in to_wp if w[2] in poheights]   # why did not match????? 
        wps.sort(key=lambda s: s[2], reverse=True)
        self.waypointSeq.append(wps[0])
        Reward,SReward, Penalty, _=self.FastStateTrans(wps[0])
        return Reward, SReward, Penalty
    def CheckReturn(self, wpc,mission):  # check it this waypoint is valid 
        if not self.Connection[wpc]:
            Delay=0
            if self.wp!=wpc:
                Delay=(self.DisMatrix[self.wp].get(wpc))/self.drone.speed+self.drone.loiter
            #print(f"check {wpc}")
            time, _=self.ShortUpload.get(wpc)
            #print(f"see {time} {gowp} {Delay}")
            Delay=Delay+time
            #print(f"see deadline mission {mission}")
            if (mission[1] -self.time)<Delay:      # Maybe storage is empty, but go to  waypoint out of connection
                print(f" Return! fly is late {mission[0]} {mission[1] -self.time} < {Delay}")
                tt, nwpc=self.ShortUpload.get(self.wp) # self.wp must be not connected, as we have 
                print(f"see return time {tt}") 
                return True
            # else:
            #     print(f"not late: {mission[0]} {mission[1] -self.time} > {Delay} {time} ?  ")
            for m, tt in self.Mission_SR.items():
                if len(tt)>0 and (self.Mission_DL[m] -self.time)<Delay: # If drone has storage, then check 
                    print(f"Need to return {Delay} < {self.Mission_DL[m]-self.time}")
                    tt, nwpc=self.ShortUpload.get(self.wp) # self.wp must be not connected, as we have 
                    print(f"see this time {tt} ") 
                    return True
                   ############## Here to see something if fly there cannot upload back! 
                   # I need to know, which mission it works and to see if it can go back!!!!! 
                   # So, I need another method to quantify the reward of selected waypoint!!!!! 
                   # I shouldn't only have one option!!!!! I need to search in the neighbor!!! 
                
        return False
        
    def Single_Step_Valid_Decision(self):
        areas=[]
        dlist=list(self.Mission_DL.items())  # Check the dl! 
        dlist.sort(key=lambda s: s[1]) # Get the earliest deadline. 
        #print(f"see mission {dlist}")
        for m in dlist:
            if len(self.Mission_JR[m[0]][0])>0:   # check which mission has this deadline? 
                areas=self.Mission_JR[m[0]][0] # Get the task areas 
                #print(f"see the deadline! {m}")
                for key, aa in self.Mission_SR[m[0]].items():
                    areas=list(set(areas)-set(aa)) # if it is stored? 
                if len(areas)>0:    
                    break 
        #print(f"Which mission time {self.time} {m[0]}")
        Diss=[self.Distance(self.wp, (a[0],a[1],0)) for a in areas]
        adex=Diss.index(min(Diss)) 
        near_a=areas[adex]         # Get the closest task area. 
        to_wp=self.Area_WPC.get(near_a)  # Get the waypoint can cover the area
        #print(f"which a {near_a}")
        poheights=list(self.drone.WPCinfo.get(m[0][0]).keys()) 
        wps=[w for w in to_wp if w[2] in poheights] 
        wps.sort(key=lambda s: s[2], reverse=True) # Choose the waypoint height! 
        ############# Check if wps[0] valid????
        Return=self.CheckReturn(wps[0],m)
        #nwpc=wps[0]
        if Return:
            nwpc=self.ShortUpload.get(self.wp)[1]
        else:
            nwpc=wps[0]
        self.waypointSeq.append(nwpc)
        Reward,SReward, Penalty,_=self.FastStateTrans(nwpc)
        return Reward, SReward, Penalty
    
    def More_Step_Valid_Decision(self):   # Consider all missions with the earliest deadline
        areas=[]
        dlist=list(self.Mission_DL.items())  # Check the dl! 
        dlist.sort(key=lambda s: s[1]) # Get the earliest deadline. 
        #print(f"see mission {dlist}")
        GetDL=None
        for m in dlist:
            if GetDL!=None:
                if m[1]==GetDL and len(self.Mission_JR[m[0]][0])>0:
                    tmp=self.Mission_JR[m[0]][0]
                    for key, aa in self.Mission_SR[m[0]].items():
                        tmp=list(set(tmp)-set(aa)) # if it is stored? 
                    if len(tmp)>0:   
                         areas=list(set(areas+tmp))
            else:
                if len(self.Mission_JR[m[0]][0])>0:   # check which mission has this deadline? 
                    areas=self.Mission_JR[m[0]][0] # Get the task areas 
                    #print(f"see the deadline! {m}")
                    for key, aa in self.Mission_SR[m[0]].items():
                        areas=list(set(areas)-set(aa)) # if it is stored? 
                    if len(areas)>0:    
                        GetDL=m[1]
                        mission=m
        Diss=[self.Distance(self.wp, (a[0],a[1],0)) for a in areas]
        adex=Diss.index(min(Diss)) 
        near_a=areas[adex]         # Get the closest task area. 
        to_wp=self.Area_WPC.get(near_a)  # Get the waypoint can cover the area
        ######## Get which mission this area have? 
        for m,st in self.TaskState[near_a].items():
            #mission,offset=m
            jr,sr=st
            if jr==0 and sr==0:
                #print(f"why fial {m}")
                mission=(m,GetDL)
        poheights=list(self.drone.WPCinfo.get(mission[0][0]).keys()) 
        wps=[w for w in to_wp if w[2] in poheights] 
        wps.sort(key=lambda s: s[2], reverse=True) # Choose the waypoint height! 
        ############# Check if wps[0] valid????
        Return=self.CheckReturn(wps[0],mission)
        #nwpc=wps[0]
        if Return:
            nwpc=self.ShortUpload.get(self.wp)[1]
        else:
            nwpc=wps[0]
        self.waypointSeq.append(nwpc)
        Reward,SReward, Penalty,_=self.FastStateTrans(nwpc)
        return Reward, SReward, Penalty
    
    def Cluster_Scheduling(self): 
        #self.DrawWPCs()
        self.waypointSeq.append(self.wp)
        Reward=0; P=0
        print(f" length", len(self.WPCHeight))   # WPCs' key is the height: waypoints; Cover_map's key is waypoint: areas
        self.initialTask()
        self.ClusterTask()
        st=time.time()
        while self.time<=1800:
            #areas=[]
            sr=DecomposeSize/Res
            dlist=list(self.Mission_DL.items())  # Check the dl! 
            dlist.sort(key=lambda s: s[1]) # Get the earliest deadline. 
            
            
            
            
            
            
            #reward, sreward, penal=self.MissionEDF_Decision()
            #reward, sreward, penal=self.Single_Step_Valid_Decision() #Check Single-step Checking
            #reward, sreward, penal=self.More_Step_Valid_Decision() #Check Single-step Checking
            reward, sreward, penal,UpdateGrid=self.Cluster_Valid_Decision(200,10,UpdateGrid) #Check Single-step Checking
            Reward=Reward+reward
            P=P+penal
        print(f"Planning time {time.time()-st}")    
        print(f" drone {self.drone.id} waypoints {Reward}, {P}, {len(self.waypointSeq)} {self.waypointSeq}")
        self.DrawWPsequence()
    
    def Cluster_Valid_Decision(self, DecomposeSize, Res,UpdateGrid):   # Check the missions in the bigger cluster!
        areas=[]
        sr=DecomposeSize/Res
        dlist=list(self.Mission_DL.items())  # Check the dl! 
        dlist.sort(key=lambda s: s[1]) # Get the earliest deadline. 
        if UpdateGrid:
            GetDL=None
            for m in dlist:
                if GetDL!=None:
                    if m[1]==GetDL and len(self.Mission_JR[m[0]][0])>0:
                        tmp=self.Mission_JR[m[0]][0]
                        for key, aa in self.Mission_SR[m[0]].items():
                            tmp=list(set(tmp)-set(aa)) # if it is stored? 
                        if len(tmp)>0:   
                             areas=list(set(areas+tmp))
                else:
                    if len(self.Mission_JR[m[0]][0])>0:   # check which mission has this deadline? 
                        areas=self.Mission_JR[m[0]][0] # Get the task areas 
                        for key, aa in self.Mission_SR[m[0]].items():
                            areas=list(set(areas)-set(aa)) # if it is stored? 
                        if len(areas)>0:    
                            GetDL=m[1]
                            mission=m
            ############ If Grid_tocov should maintain all tasks? If not, we do not need it every time!!! 
            CurGrid=set()
            for a in areas: 
                self.Grid_tocov[self.Area_grid[a]].append(a)
                CurGrid.add(self.Area_grid[a])
            #Select a grid to cover! 
            ###################################################
            CurGrid=list(CurGrid)
            print(f"see waypoint {self.wp} {CurGrid}")
            Diss=[self.Distance(self.wp, (g[0][0]*sr,g[0][1]*sr,0)) for g in CurGrid]
            adex=Diss.index(min(Diss)) 
            near_g=CurGrid[adex] 
            print(f"see near Grid {near_g}")
        else:
            if len(self.Area_grid[near_g])==0:     #------>  We need update self.Area_grid during state transition!! 
                #Change another grid, 
                CurGird.remove(near_g)
                Diss=[self.Distance(self.wp, (g[0][0]*sr,g[0][1]*sr,0)) for g in CurGrid]
                adex=Diss.index(min(Diss)) 
                near_g=CurGrid[adex] 
        # ########################### Select waypoint within the grid!!!
        # Diss=[self.Distance(self.wp, (a[0],a[1],0)) for a in areas]
        # adex=Diss.index(min(Diss)) 
        # near_a=areas[adex]         # Get the closest task area. 
        # to_wp=self.Area_WPC.get(near_a)  # Get the waypoint can cover the area
        # ######## Get which mission this area have? 
        # for m,st in self.TaskState[near_a].items():
        #     #mission,offset=m
        #     jr,sr=st
        #     if jr==0 and sr==0:
        #         #print(f"why fial {m}")
        #         mission=(m,GetDL)
        # poheights=list(self.drone.WPCinfo.get(mission[0][0]).keys()) 
        # wps=[w for w in to_wp if w[2] in poheights] 
        # wps.sort(key=lambda s: s[2], reverse=True) # Choose the waypoint height! 
        ##################################################################################
        ############# Check if wps[0] valid????
        Return=self.CheckReturn(wps[0],mission)
        #nwpc=wps[0]
        if Return:
            nwpc=self.ShortUpload.get(self.wp)[1]
        else:
            nwpc=wps[0]
        self.waypointSeq.append(nwpc)
        Reward,SReward, Penalty,UpdateGrid=self.FastStateTrans(nwpc)
        return Reward, SReward, Penalty
        
    
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
    speeds=[10,5,5,5,5,5]
    loiter=[1,2,1,1,1]
    sensgrp=[['ZENMUSE_XT2_t','ZENMUSE_XT2_r'],['DJI_Air2S'],['ZENMUSE_XT2_t','ZENMUSE_XT2_r']]
    Drones=LoadDrones(sensorfile,PPMfile,DroneNum, speeds, sensgrp, Res,loiter)
    ##################################################### Task Allocation 
    Drones,Bidders=Auction(AreaPara, Drones, Res,Missions,Task_mission,1)
    p=TrackDraw(Drones, BSshp, AreaPara)
    for drone in Drones:
        planner=FlightPlanner(drone,init=init, planTime=10, iniloc=(50,0,0), GCloc=(50,0,0),Missions=Missions,Res=Res, DecomposeSize=DecomposeSize) # Here, GCloc and iniloc divided by Res!
        planner.GenWaypoint()
        planner.Scheduling()
        #TrackDraw(Drones, BSshp, AreaPara)
        #imshow(p)
        #plt.show()
    # def EDF_Decision(self): #select waypoint with earliest deadline
    #     #st=time.time()
    #     State_Dict=defaultdict(list)
    #     #print(f"check time {len(self.TaskState.items())}")
    #     for i in self.TaskState.items():   # every time go over this! too slow! 
    #         for ts in i[1]:
    #             State_Dict[ts].append(i[0])
    #     # Do we need check this every time?????? 
    #     #print(f"see State {ts} {State_Dict[ts]}")  
    #     ts=list(State_Dict.keys())
    #     ts.sort(key=lambda s: s[1])        
    #     areas=State_Dict.get(ts[0])
    #     Diss=[self.Distance(self.wp, (a[0],a[1],0)) for a in areas]
    #     near_a=[a for a in areas if self.Distance(self.wp, (a[0],a[1],0))==min(Diss)][0]
    #     to_wp=self.Area_WPC.get(near_a)
    #     to_wp.sort(key=lambda s: s[2], reverse=True)
    #     self.waypointSeq.append(to_wp[0])
    #     self.FastStateTrans(to_wp[0])
    #     #print(f"see EDFdecision {time.time()-st} ")
    #     #print(f"see time {time.time()-st} ")
    #     #self.TrackState()

    
    












