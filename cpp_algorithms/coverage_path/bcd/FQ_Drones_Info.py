
import numpy as np
import matplotlib.pyplot as plt
import sys
import geopandas as gpd
import pandas as pd
#from cpp_algorithms.common_helpers import imshow, imshow_scatter
from bcd_helper import imshow, imshow_scatter
import math
from collections import defaultdict
import os
import random
from FQ_GenTask import GetEFFNext, UpadteEFA, logEFA,GenTasks
from geopandas._vectorized import length


class Sensor:
    def __init__(self, id=1, name=" ", type='ALL', FOV_H=72, FOV_V=58,I_H=3840, I_V=2160, size=4):
        self.id=id
        self.name=name
        self.type=type
        self.FOV_H=FOV_H
        self.FOV_V=FOV_V
        self.I_H=I_H
        self.I_V=I_V
        self.size=size
    def PPM_to_H(self,PPM,max_H=120, min_H=25):
        width=self.I_H/PPM
        h=width/(2**math.tan(self.FOV_H*math.pi/360))
        h=np.floor(h)
        if h>max_H:
            h=max_H
        length=2*h*math.tan(self.FOV_V*math.pi/360)
        width=2*h*math.tan(self.FOV_H*math.pi/360)
        FOV=min(length,width)
        if h<min_H:
            return False,0
        return h,FOV

class Drone:
    def __init__(self, sensors=[], id=0,range=500, speed=3, loiter=1): # sensor can be: sensor=['RGB', 'THM', 'All']
        self.id=id
        self.range=range
        self.sensortype=''
        self.sensors=[]
        self.speed=speed
        self.maxH=120
        self.loiter=loiter
        self.minH=25
        self.area=[]
        self.iniloc=[]
        self.WPCinfo={}
    def AddSensors(self,sensors):
        self.sensors=self.sensors+sensors
        #self.sensors.append(sensor)
        if len(self.sensors)==1:
            self.sensortype=sensors[0].type
        if len(self.sensors)==2:
            self.sensortype='ALL'
    def AddRules(self,maxH,minH):
        self.maxH=maxH
        self.minH=minH
    def CombineHeight(self,HSset): # here we consider 2 kind of sensors
        delete=[]
        for i in range(len(HSset)):
            for h in HSset[i]:
                for j in list(set(range(len(HSset)))-set([i])):
                    hh=[h1 for h1 in list(HSset[j]) if h1>=h] 
                    hl=[h1 for h1 in list(HSset[j]) if h1<h] 
                    if len(hh)>0:
                        mh=min(hh )
                        tmp0=HSset[i].get(h)
                        tmp1=HSset[j].get(mh)
                        if tmp1[0]>=tmp0[0] and tmp1[1]>=tmp0[1]:
                            delete.append([i,h])
                            #print(f"see hh {i} {tmp0} {tmp1}")
                    if [i,h] not in delete and len(hl)>0:
                        mh=max(hl)
                        tmp0=HSset[i].get(h)
                        tmp1=HSset[j].get(mh)
                        if tmp1[0]>=tmp0[0] and tmp1[1]>=tmp0[1]:
                            delete.append([i,h])
        for d in delete:
            del HSset[d[0]][d[1]]
        FHSset={}
        for i in range(len(HSset)):
            for k in HSset[i]:
                FHSset[k]=HSset[i][k]
        return FHSset
    def GetFOV(self,MisPPM,Res):# Now only consider single drone covers the whole area
        WPCinfo=defaultdict(dict)
        for m in MisPPM:
            HSset=[]
            for s in self.sensors:
                #M_Hdict=defaultdict(dict) # key is the mission:{ height: [reward, WPC, COV]}
                H_FRdict={}
                if MisPPM[m].get(s.type)!=None:
                    tmp=MisPPM[m].get(s.type)
                    for ppm in tmp:
                        h,fov1=s.PPM_to_H(ppm,self.maxH,self.minH)
                        fov=np.floor(fov1/Res)*Res
                        if h!=False:
                            H_FRdict[h]=[fov, tmp[ppm],s.type] # FOV, Reward, type
                if len(H_FRdict)>0:
                    HSset.append(H_FRdict)
            if len(HSset)>1:
                H_FRdict=self.CombineHeight(HSset)
            elif len(HSset)==1:
                H_FRdict=HSset[0]
            #print(f"check {H_FRdict}")
            WPCinfo[m]=H_FRdict
        self.WPCinfo=WPCinfo
        return WPCinfo
    def GenWPCandidate(self,EFA,Res,Task_mission,WPCinfo): ## it is incorrect!!!! 
        WPCdict=defaultdict(dict)
        Wdict=defaultdict(dict) # key is the mission
        for m in missions:
            H_FRdict=WPCinfo[m]
            for h in H_FRdict: #traverse the height
                fov=H_FRdict[h][0]
                sh=np.array(EFA.shape)*Res/fov    ##############
                for i in range(2):
                    sh[i]=np.ceil(sh[i])
                sh=np.int64(sh)
                WPC=[[[] for i in range(sh[1])]for i in range(sh[0])]
                COV=[[[] for i in range(sh[1])]for i in range(sh[0])]
                for i in range(sh[0]):
                    for j in range(sh[1]):
                        WPC[i][j]=[fov*i+fov/2, fov*j+fov/2]############ Later we also need to consider the partial coverage
                        COV[i][j]=[[(fov/Res)*i, (fov/Res)*(j)], [(fov/Res)*(i+1), (fov/Res)*(j+1)]]
                        #WPCset.append([fov*i+fov/2, fov*j+fov/2])  
                #print(len(WPCset))
                Wdict[h]=[WPC,COV]
            WPCdict[m]=Wdict
        return WPCdict
        # plt.scatter([WPCset[i][0] for i in range(len(WPCset))], [WPCset[i][1] for i in range(len(WPCset))])
        # plt.show()
        #return WPC, WPCset
def ReadSen_PPM(sensorfile,PPMfile):
    df=pd.read_csv(sensorfile,delimiter=r"\s+")
    Sensors={}
    for i in range(len(df)):
        ob=df.loc[i]
        s=Sensor(id=i, name=ob['name'], type=ob['type'], FOV_H=ob['FOV_H'], FOV_V=ob['FOV_V'],I_H=ob['I_H'], I_V=ob['I_V'], size=ob['size'])
        Sensors[i]=s
        Sensors[ob['name']]=s # We use both id and name as the index! 
    PPMdf=pd.read_csv(PPMfile,delimiter=r"\s+")
    MisPPM=defaultdict(dict)
    for i in range(len(PPMdf)):
        ob=PPMdf.loc[i]
        ppmlist=ob['PPM'].split(',')
        rwlist=ob['reward'].split(',')
        tmp={}
        for j in range(len(ppmlist)):
            tmp[float(ppmlist[j])]=float(rwlist[j])
        MisPPM[ob['mission']][ob['sensor']]=tmp
    return Sensors,MisPPM
def LoadDrones(sensorfile,PPMfile,DroneNum, speeds, sensgroup, Res, Loiter):
    Sensors,MisPPM=ReadSen_PPM(sensorfile,PPMfile)
    Drones=[]
    for i in range(DroneNum):
        drone=Drone(id=i,speed=speeds[i],loiter=Loiter[i])
        sens=[Sensors[s] for s in sensgroup[i]]
        drone.AddSensors(sens)
        drone.GetFOV(MisPPM,Res)
        Drones.append(drone)
    return Drones

if __name__ == "__main__":
    Missions=defaultdict(dict)  #  Missions: id, period, priority, 
    Missions['BM']=[ 10, 1]  #Other area 
    Missions['FI']=[ 5, 1]  #track the fire 
    Missions['FT']=[ 3, 3]  # track the fire perimeter and the risky area (arrival within 10 min)
    Missions['FL']=[ 3, 2]  # FL is tracking the fire perimeter.
    Missions['FD']=[ 2, 3]
    dir='/home/fangqiliu/eclipse-workspace_Python/Drone_path/CoveragePathPlanning-master/farsite/'
    foldername='FQ_burn'
    # BunsiteBound=(701235.0,4308525.0,704355.0,4311675.0)
    # BunsiteBound=(702374.0,4309425.0,703700.0,4310900.0)
    Bursite=(702460.0,4309700.0,703540.0,4310820.0 )
    init=120; end=150; Res=10
    Task_mission=GenTasks(init,end,Bursite,dir,foldername,Missions, Res=Res)
    ##########################################################
    sensorfile='Data/sensor_info.csv'
    PPMfile='Data/PPM_table.csv'
    Sensors,MisPPM=ReadSen_PPM(sensorfile,PPMfile)
    # drone=Drone(id=0)
    # drone.AddSensors([Sensors['DJI_Air2S']])
    # drone1=Drone(id=1)
    # drone1.AddSensors([Sensors['ZENMUSE_XT2_t'],Sensors['ZENMUSE_XT2_r']])
    # drone1.GetFOV(MisPPM, Res)
    DroneNum=3
    speeds=[5,5,5,5,5,5]
    loiter=[1,2,1,1,1]
    sensgrp=[['ZENMUSE_XT2_t','ZENMUSE_XT2_r'],['DJI_Air2S'],['ZENMUSE_XT2_t','ZENMUSE_XT2_r']]
    Drones,Bidders=Auction(AreaPara, Drones, Res,Missions,Task_mission,1)
















