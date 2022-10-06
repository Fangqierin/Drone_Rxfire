
import numpy as np
import matplotlib.pyplot as plt
import sys
import geopandas as gpd
from _collections import defaultdict
sys.path.append('../../../')
#from cpp_algorithms.common_helpers import imshow, imshow_scatter
from bcd_helper import imshow, imshow_scatter, cluster_imshow
from shapely.geometry import Point, Polygon
from pathlib import Path
# from zipfile import ZipFile
#from geopy import distance
#from skimage import measure
import math
from skimage.draw import polygon
import shapely
import shapely.geometry
from collections import defaultdict
from skimage.draw import polygon
import os
import random
from FQ_GenTask import GetEFFNext, UpadteEFA, logEFA
from FQ_Drones_Info import Sensor, Drone, ReadSen_PPM, LoadDrones

def Voronoi(EFA):    # decomposition using Voronoi method
    Grid={}
    Num=20
    p=np.full((len(EFA),len(EFA[0])), -1)
    # suppose 4 groups
    xn=random.sample(range(len(EFA)), Num)
    yn=random.sample(range(len(EFA[0])), Num)
    Group=defaultdict(list) 
    for i in range(Num):
        Group[i].append((xn[i],yn[i]))
    for i in range(len(EFA)):
        for j in range(len(EFA[i])):
            distances=[(i-xn[k])**2+(j-yn[k])**2 for k in range(Num)]
            C= distances.index(min(distances))
            p[i][j]=C
    #imshow(p)
    #plt.scatter(xn,yn, color='black')
    #plt.show()
    return p, xn,yn
def Decomposition(DecomposeSize, Res,Task_mission):  # Return the dictionary of grids and area
    DecomposeSize=int((DecomposeSize/Res)*Res)
    sc=DecomposeSize/Res
    AreaPara=defaultdict(dict)
    p=np.full((len(Task_mission['BM'][1]),len(Task_mission['BM'][1][0])), -1)
    cc=0
    #examples: Taskdict['FT']=[init, FTA,Missions['FT'][0],Missions['FT'][1]]
    tmp=list(Task_mission.keys())[0]
    perTable=np.full((len(Task_mission[tmp][1]),len(Task_mission[tmp][1][0])), 255)
    Task_mission=sorted(Task_mission.items(), key=lambda x:x[1][2])
    #print(f"Check {Task_mission}")
    #Here is tricky, I sort it by its period, so it grid will not overlapped
    #Every grid, we only consider the minimum period, right! 
    cc=0
    for mm in Task_mission:
        period=mm[1][2]
        cov=mm[1][1]
        Grid_mission=defaultdict(list)
        Area=defaultdict(list)
        x,y =np.where(cov==1)
        for i in range(len(x)):
            if perTable[x[i],y[i]]>period:
                ######### We will prioritize the short period! 
                perTable[x[i],y[i]]=period
                xc=int(x[i]/sc); yc=int(y[i]/sc)
                Area[(xc,yc)].append([x[i],y[i],mm[1][0]])
                p[x[i],y[i]]=cc#xc+yc+cc*2
        AreaPara[mm[0]]=Area
        cc=cc+1
    # cluster_imshow(p,sc=sc)
    # plt.show()
    return AreaPara

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
    #plt.show()  
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
    grids=[(i[0],i[1],i[-1]) for i in Grids]
    gridBM=[i for i in grids if i[0]=='BM']
    gridF=[i for i in grids if i[0]!='BM']
    MaxRange=Res*max([Distance(i[-1],GCloc) for i in grids])
    #MaxBM=max([Distance(i[-1],GCloc) for i in gridBM])
    dronestate=[(B.id, B.sensortype, max(0,MaxRange-B.range)/B.speed) for B in Bidders] #Smaller is better
    donBM=sorted([i for i in dronestate if i[1]=='RGB'], key=lambda x: x[2], reverse=True) #long --> short
    donF=sorted([i for i in dronestate if i[1]!='RGB'], key=lambda x: x[2],reverse=True)
    asgG=[];loc={}
    asgBM=[]; asgF=[]
    random.seed(sed)
    #print(f"see gridF {gridF} {GCloc}")
    for i in range(len(donF)):
        if i==0:
            #m_d=random.choice(range(len(gridF)))
            Dis=[Distance(g[-1],GCloc) for g in gridF]
        elif len(donF)>1 and i==1:
            Dis=[-1*Distance(g[-1],GCloc) for g in gridF]
           # print()
           #print(f"see Dis {Dis} {max(Dis)} {gridF[Dis.index(max(Dis))]} ")
        else:
            Dis=[sum([Distance(g[-1],j[-1]) for j in asgG]) for g in gridF]
        m_d=Dis.index(max(Dis))
        asgG.append(gridF[m_d])
        asgF.append(gridF[m_d])
        gridF.remove(gridF[m_d]) 
    for i in range(len(donBM)):
        if i==0:
            Dis=[Distance(i[-1],GCloc) for i in gridBM]
        else:
            Dis=[sum([Distance(i[-1],j[-1]) for j in asgG]) for i in gridBM]
        m_d=Dis.index(max(Dis))
        loc[donBM[i][0]]=gridBM[m_d]
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
    #print(f"see loc {loc}")
    return loc

def RangeLocDrone(Bidders, GC, GCloc, Grids, sed, Res):
    DNum=len(Bidders)
    dronestate=[(B.id, B.sensortype, B.range/B.speed) for B in Bidders]
    donBM=sorted([i for i in dronestate if i[1]=='RGB'], key=lambda x: x[2]) #long --> short
    donF=sorted([i for i in dronestate if i[1]!='RGB'], key=lambda x: x[2])
    def Distance(x,y):
        #print(f"x, y {x} {y}")
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
            #Dis=[Distance(i[-1],GCloc) for i in gridF]
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
    #print(GridsBM, GridsF)
    for i in range(len(GridsBM)):
        loc[donBM[i][0]]=GridsBM[i][0]
    for i in range(len(GridsF)):
        loc[donF[i][0]]=GridsF[i][0]
    #print(f"see loc {loc}")
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
    def __init__(self, Drone, GCloc, Res): # sensor can be: sensor=['RGB', 'THM', 'All']
        self.id=Drone.id
        self.range=Drone.range
        self.sensortype=Drone.sensortype
        self.speed=Drone.speed
        self.FoVinfo=Drone.WPCinfo  # This should be computed by PPM!!!
        self.reward=defaultdict(dict)
        self.assignGrid=[]
        self.curcost=[]
        self.flytime=0
        self.coverarea=0
        self.area=[]
        self.iniArea=[]
        self.GCloc=GCloc
        self.Res=Res
        self.DisMission={} # maintain the minimal distance 
        self.DisMax=None
        self.UploadU=0
        self.MaxUpload=0
        self.MissionSet=set()
    def GetDistance(self,g1,g2):
        return math.sqrt((g1[-1][0]-g2[-1][0])**2+(g1[-1][1]-g2[-1][1])**2)
    #math.sqrt((x[0]-GCloc[0])**2+(x[1]-GCloc[1])**2)
    def GetCost(self, grid):
        #print(f"see grid {grid}")
        if len(self.FoVinfo.get(grid[0]))==0:
            return 10000000000000
        FOV=max(list(self.FoVinfo.get(grid[0]).keys()))
        ##return self.Gettravetime(grid)
        period=grid[-2]*60
        travel=self.Gettravetime(grid)
        if len(self.area)==0:
             return travel  ################ FQ, should we add initial location????????????????
        if travel>1 and len(self.area)>0:
            #print(f"why {grid[2]} {FOV} {period} {self.speed}")
        #Here we need add travel time! 
            return 0.1+self.coverarea+(grid[2]/(self.speed*FOV*period))
        if travel==0 and len(self.area)>0:
            return 0
        # if len(self.area)==0:
        #     return self.coverarea+(grid[2]/(self.speed*self.FoV*period))+travel
        #print(f"see {self.coverarea} {self.coverarea+(grid[2]/(self.speed*FOV*period))}")
        return self.coverarea+(grid[2]/(self.speed*FOV*period))#+travel/self.speed
    def GetCost_New(self, grid):
        if len(self.FoVinfo.get(grid[0]))==0:
            return 10000000000000
        FOV=max(list(self.FoVinfo.get(grid[0]).keys()))
        period=grid[-2]*60
        travel=self.GetTravetime(grid)
        #Back=max(0,self.Res*math.sqrt((grid[-1][0]-self.GCloc[0])**2+(grid[1][1]-self.GCloc[1])**2)-self.range)
        ######### Compute Back travel time! 
        traverse=self.TraverseTime(grid) # minimal distance to assigned grids. 
        travelU=self.flytime+(traverse*self.Res)/(self.speed*period)
        #UploadU=self.GetUploadTime(grid)
        UploadU=self.MaxUploadTime(grid)
        if travel==0:
            #return 0+self.coverarea+travelU+(grid[2]/(self.speed*FOV*period))+UploadU
            #return 0+self.coverarea+travelU#+self.GetUploadTime(None)#+UploadU#+(grid[2]/(self.speed*FOV*period))
            #return 0+self.coverarea+travelU#+self.GetUploadTime(None)#+UploadU#+(grid[2]/(self.speed*FOV*period))
            return 0+self.coverarea+travelU+self.MaxUploadTime(None)#+(grid[2]/(self.speed*FOV*period))
        # if travel>1:
        #     return self.coverarea+(grid[2]/(self.speed*FOV*period))+travelU#+
        return self.coverarea+travelU+(grid[2]/(self.speed*FOV*period))+self.MaxUploadTime(None)#self.GetUploadTime(None)#+(grid[2]/(self.speed*FOV*period))#+UploadU#+travel/self.speed
    def GetTravetime(self,grid):   # Cell distance
        if len(self.assignGrid)==0:
            return math.sqrt((grid[1][0]-self.iniArea[1][0])**2+(grid[1][1]-self.iniArea[1][1])**2)
        add=min([math.sqrt((grid[1][0]-k[0])**2+(grid[1][1]-k[1])**2) for k in self.assignGrid])
        return add
    def TraverseSum(self,grid):   # Cell distance
        if len(self.assignGrid)==0:
            return 0#math.sqrt((grid[1][0]-self.iniArea[1][0])**2+(grid[1][1]-self.iniArea[1][1])**2)
        add=sum([self.GetDistance(grid, k) for k in self.area])
        return add
    def TraverseTime(self,grid): # Cell Center distance
        #Grids.append([mm, cell, len(grids)*(Res**2),grids,period,(cx,cy)])
        if len(self.assignGrid)==0:
            return self.GetDistance(grid, self.iniArea)#+ math.sqrt((grid[-1][0]-self.GCloc[0])**2+(grid[1][1]-self.GCloc[1])**2)
        travel=self.GetTravetime(grid)
        if travel==0:
            return 0
        add=min([self.GetDistance(grid, k) for k in self.area])
        #print(f"why distance {add} {[self.GetDistance(grid, k) for k in self.area]}")
        return add
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
                #UploadU=UploadU+ (2*max(0,mind-self.range))/(self.speed*p)
        if mission not in list(self.DisMission.keys()):
            UploadU=UploadU+ (max(0,diss-self.range))/(self.speed*period)
        return UploadU
    
    def MaxUploadTime(self,grid):
        if grid==None:
            try:
                minp, maxd,mind=self.DisMax
                UploadU= (max(0,maxd-self.range)+max(0,mind-self.range))/(self.speed*minp)
                return UploadU
            except:
                return 0
        period=grid[-2]*60
        diss=math.sqrt((grid[-1][0]-self.GCloc[0])**2+(grid[1][1]-self.GCloc[1])**2)*self.Res
        if len(self.area)>0:
            minp, maxd,mind=self.DisMax
            period=min(minp,period);maxd=max(diss,maxd);mind=min(diss,mind)
        else:
            period=period; maxd=diss; mind=diss
        UploadU=(max(0,maxd-self.range)+max(0,mind-self.range))/(self.speed*period)
        #print(f"see Upload", UploadU)
        return UploadU
    
    def AddGrid(self, grid):
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
        self.coverarea=self.coverarea+(grid[2]/(self.speed*FOV*period))  # add area
        self.flytime=self.flytime+ self.TraverseTime(grid)*self.Res/(self.speed*period)
        #print(f"why flytime {self.TraverseTime(grid)*self.Res} {(self.speed*period)} ")
        self.assignGrid.append(grid[1])
        self.area.append(grid)
        self.MissionSet.add(grid[0])
        self.UploadU=self.GetUploadTime(None)
        self.MaxUpload=self.MaxUploadTime(None)
        
def Bidding(Grids, Bidders, EFAM):
    # print(f"see Grids")
    # for g in Grids:
    #     print(g[-1])
    while len(Grids)>0: 
        bid=[];costs=[]
        for i in range(len(Bidders)):
            #costs.append([Bidders[i].GetCost(g) for g in Grids])
            costs.append([Bidders[i].GetCost_New(g) for g in Grids])
        bid=[min(k) for k in costs]
        price=min(bid)
        winners=[k for k in range(len(bid)) if bid[k]==price]
        #print(f"see winner {winners} ")
        rm=[]
        #print(f"bid {bid}")
        # print(f"win {winners}")
        # print(f"bid {bid}")

        for j in winners:   # once have multiple winner or one winner????? 
            nums=[k for k in range(len(costs[j])) if costs[j][k]==price and Grids[k] not in rm]
            if len(nums)>1:
                #print(f"win {winners}")
                #print(f"see grid {[Grids[i] for i in nums]}")
                srnum=sorted([(i, Bidders[j].TraverseSum(Grids[i])) for i in nums], key=lambda x: x[1])
                nums=[s[0] for s in srnum]
                #print(f"see ", srnum)
                #print(f"seeesss {nums}")
                #TrackDraw(Bidders,EFAM)
            if len(nums)>0:
                Bidders[j].AddGrid(Grids[nums[0]])
                rm.append(Grids[nums[0]])
        for r in rm:
            Grids.remove(r)
        # print(f"see Drones flytime {[Bidders[i].flytime for i in range(len(Bidders))]}")
        # print(f"see Drones cost {[Bidders[i].coverarea for i in range(len(Bidders))]}")
        # print(f"see Drones upload {[Bidders[i].UploadU for i in range(len(Bidders))]}")
        # print(f"see Drones maxupload {[Bidders[i].MaxUpload for i in range(len(Bidders))]}")
        # print(f"see Drones cost {[Bidders[i].coverarea+Bidders[i].flytime+Bidders[i].MaxUpload for i in range(len(Bidders))]}")
        # print(f"see Drones cost {[Bidders[i].coverarea+Bidders[i].flytime+Bidders[i].UploadU for i in range(len(Bidders))]}")

        #print(f"winner {winners} {len(Grids)}")
        #TrackDraw(Bidders,EFAM)
    return Bidders
def Auction(AreaPara, Drones, Res,Missions, seed, EFAM, GCloc):
    # print(f"check AreaPara")
    # for key, grids in AreaPara.items():
    #     print(key, grids)
    GCloc=np.array(GCloc)//Res
    print(f"see GCloc {GCloc}")
    Bidders=[]
    for d in Drones:
        Bidders.append(Bidder(d,GCloc, Res))
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
        
        Grids.append([mm, cell, len(grids)*(Res**2),grids,period,(cx,cy)])
        # if mm=='BM':
        #     AGrids.append([mm, cell, len(grids)*(Res**2),grids,period,(cx,cy)])
        # else:
        #     FGrids.append([mm, cell, len(grids)*(Res**2),grids,period,(cx,cy)])
    #loc=HisLocDrone(Bidders,seed, GCloc, Grids, seed)
    loc=ExtrmLocDrone(Bidders,seed, GCloc, Grids, seed, Res)
    for i in range(len(Bidders)):
        Bidders[i].iniArea=loc[i]
    Bidders=Bidding(Grids,Bidders, EFAM)
    #########Fire first, then others
    #Bidders=Bidding(FGrids,Bidders, EFAM)
    #TrackDraw(Bidders,EFAM)
    #Bidders=Bidding(AGrids,Bidders, EFAM)
    for i in range(len(Bidders)):
        Drones[i].area=Bidders[i].area
        Drones[i].MissionSet=Bidders[i].MissionSet
        #print(f"check what is area {Drones[i].area}")
    return Drones, Bidders
if __name__ == "__main__":
    Missions=defaultdict(dict)  #  Missions: id, period, priority, 
    Missions['BM']=[ 10, 1]  #Other area 
    Missions['FI']=[ 5, 1]  #track the fire 
    Missions['FT']=[ 3, 3]  # track the fire perimeter and the risky area (arrival within 10 min)
    Missions['FL']=[ 3, 2]  # FL is tracking the fire perimeter.
    Missions['FD']=[ 2, 3]
    ################ Get tasks.
    dir='/home/fangqiliu/eclipse-workspace_Python/Drone_path/CoveragePathPlanning-master/farsite'
    foldername='FQ_burn'
    # BunsiteBound=(701235.0,4308525.0,704355.0,4311675.0)
    # BunsiteBound=(702374.0,4309425.0,703700.0,4310900.0)
    Bursite=(702460.0,4309700.0,703540.0,4310820.0 )
    init=0; end=10; Res=10
    ############################Drone loading
    sensorfile='Data/sensor_info.csv'
    PPMfile='Data/PPM_table.csv'
    DroneNum=3
    speeds=[5,5,5,5,5,5]
    loiter=[1,2,1,1,1]
    GCloc=(0,0)
    sensgrp=[['ZENMUSE_XT2_t','ZENMUSE_XT2_r'],['DJI_Air2S'],['ZENMUSE_XT2_t','ZENMUSE_XT2_r']]
    Drones=LoadDrones(sensorfile,PPMfile,DroneNum, speeds, sensgrp, Res,loiter)
    Task_mission,BSshp=GenTasks(init,end,Bursite,dir,foldername,Missions, Res=Res)
    ########################## Do decomposition~ 
    DecomposeSize=10
    AreaPara=Decomposition(DecomposeSize,Res,Task_mission) # Need put tasks there!!!
    ########################## Get drones
    
    ##################################################### Task Allocation 
    Drones,Bidders=Auction(AreaPara, Drones, Res,Missions,1,GCloc)
    #print(f"see Drones cost {[Bidders[i].coverarea for i in range(len(Drones))]}")
    TrackDraw(Drones, BSshp, AreaPara)



# def Auction1(AreaPara, Drones, Res,Missions, seed):
#     Bidders=[]
#     for d in Drones:
#         Bidders.append(Bidder(Drone=d))
#     Grids=[] # Get all grids
#     for mm in AreaPara:
#         period=Missions.get(mm)[0]
#         for key in AreaPara[mm]:   #There is no overlapped area (from decomposition) 
#             Grids.append([mm, key, len(AreaPara[mm][key])*(Res**2),AreaPara[mm][key],period])
#     loc=RanLocDrone(AreaPara, Bidders,seed)
#     for i in range(len(Bidders)):
#         Bidders[i].iniArea=loc[i]
#     Bidders=Bidding(Grids,Bidders)
#     #########Fire first, then others
#     # Drones=Bidding(FGrids,Drones,Area, Fire, EFA,Tasks)
#     # Drones=Bidding(AGrids,Drones,Area, Fire, EFA,Tasks)
#     for i in range(len(Bidders)):
#         Drones[i].area=Bidders[i].area
#     return Drones, Bidders
















