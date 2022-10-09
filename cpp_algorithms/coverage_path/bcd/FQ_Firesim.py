
import numpy as np
import matplotlib.pyplot as plt
import sys
import geopandas as gpd
sys.path.append('../../../')
from cpp_algorithms.common_helpers import imshow, imshow_scatter
from shapely.geometry import Point, Polygon
from pathlib import Path
from pyproj import Transformer
# from zipfile import ZipFile
#from geopy import distance
#from skimage import measure
from skimage.draw import polygon
from shapely.geometry import Point, LineString, Polygon, MultiLineString
from shapely.ops import cascaded_union
import shapely
import shapely.geometry
from collections import defaultdict
from skimage.draw import polygon
import random 
import os
import re
import math
from time import sleep
from subprocess import PIPE, Popen
import subprocess
import json
import fiona
from shapely.geometry import shape 
import pandas as pd
import time

####################################### Rxfire
def CreateRectangle(x_dl,y_dl, weight,height):# gag 10 m
    x_r=x_dl+weight
    y_u=y_dl+height
    coords=[(x_dl,y_dl), (x_r,y_dl), (x_r,y_u), (x_dl,y_u), (x_dl, y_dl)]
    poly=Polygon(coords)
    return poly 

def ClearBarrier(data):      
    polygons=[]
    for index, row in data.iterrows():
        poly = row['geometry']
        #print(f"seeeee {poly}")
        polygons.append(poly)
    #print(f"whyyyyy {polygons[0]}")
    u=cascaded_union([polygons[0],polygons[1],polygons[2]])
    u2=cascaded_union([polygons[3],polygons[6]])
    x,y=u2.exterior.coords.xy
    newdata = gpd.GeoDataFrame()
    newdata['geometry'] = None
    newdata.loc[0, 'geometry'] = u2.boundary[0]
    newdata.loc[1, 'geometry'] = polygons[4].boundary
    newdata.loc[2, 'geometry'] = u.boundary
    #da2=gpd.read_file(f"{dir}/{foldername}/input/seelin.shp")
    return newdata
    # da2.plot()
    # plt.show()  
    
def WriteInput(foldername, dir,step=1,initime=(0,0),dura=(2,0), dis_res=5, pre_res=5, wind=5,direction=270): # The simulation time 100 =1:00) duration<24 hours!
    fin = open(f"{dir}/{foldername}/input/Burn.input", "w")
    with open(f"{dir}/Template_burn/input/Burn.input", "r") as ftmp:
        filedata = ftmp.readlines()
    checkwind=False
    FARSITE_START_TIME=''
    for line in filedata:
        if re.match(r'FARSITE_START_TIME:*',line):
            time=line[:-1].split(' ')[-1]
            checkwind=True
            if initime[0]<10:
                time=f"0{initime[0]}"
            else:
                time=f"{initime[0]}"
            if initime[1]<10:
                time=time+f"0{initime[1]}"
            else:
                time=time+f"{initime[1]}"
            FARSITE_START_TIME=r'05 04 '+f"{time}"
            fin.write(f"FARSITE_START_TIME: {FARSITE_START_TIME}\n")
        elif checkwind  and re.match(r'2013 5 4 '+f"{time}",line):
            wind=int(line[:-1].split(' ')[-3])
            winddric=int(line[:-1].split(' ')[-2])
            fin.write(line)
        elif re.match(r'FARSITE_END_TIME:*',line):
            ent=[]
            ent.append(initime[0]+dura[0])
            if initime[1]+dura[1]>60:
                ent[0]=ent[0]+1
                ent.append(initime[1]+dura[1]-60)
            else:
                ent.append(initime[1]+dura[1])
            if ent[0]<10:
                time=f"0{ent[0]}"
            else:
                time=f"{ent[0]}"
            if ent[1]<10:
                time=time+f"0{ent[1]}"
            else:
                time=time+f"{ent[1]}"
            fin.write(f"FARSITE_END_TIME: 05 04 {time}\n")
        elif re.match(r'FARSITE_TIMESTEP:*',line):
            fin.write(f"FARSITE_TIMESTEP: {step}\n")
        elif re.match(f"FARSITE_DISTANCE_RES: ",line):
            fin.write(f"FARSITE_DISTANCE_RES: {float(dis_res)}\n")
        elif re.match(f"FARSITE_PERIMETER_RES: ",line):
            fin.write(f"FARSITE_PERIMETER_RES: {float(pre_res)}\n")
        elif re.match(f"2013 5 4 0000 58 26 0.00",line):
            fin.write(f"2013 5 4 0000 58 26 0.00 {wind} {direction} 0\n")
        else:
            fin.write(line)
    fin.close()

def Addfire(foldername, dir, prefix, time, poly):   # Update the ignition file! because we add some fire!!!! 
    collection = list(fiona.open(f"{dir}/{foldername}/output/{prefix}_{time}_Perimeters.shp",'r'))
    df1 = pd.DataFrame(collection)
    def isvalid(geom):
            if len(geom['coordinates'][0])>2:
                return 1
            else:
                return 0
    df1['isvalid'] = df1['geometry'].apply(lambda x: isvalid(x))
    df1 = df1[df1['isvalid'] == 1]
    collection = json.loads(df1.to_json(orient='records'))
    #Convert to geodataframe
    data = gpd.GeoDataFrame.from_features(collection)
    try:
        print(f"{[data.loc[i, 'geometry'] for i in range(len(data))]}")
        geoms=[data.loc[i, 'geometry'] for i in range(len(data))]+[poly]
        data2=shapely.ops.unary_union([geom if geom.is_valid else geom.buffer(0) for geom in geoms]) 
    except:
        #data2=cascaded_union([data.loc[i, 'geometry'] for i in range(len(data))])
        geoms=[data.loc[i, 'geometry'] for i in range(len(data))]
        data2=shapely.ops.unary_union([geom if geom.is_valid else geom.buffer(0) for geom in geoms])
    try:
        features = [i for i in range(len(data2))]
        gdr = gpd.GeoDataFrame({'feature': features, 'geometry': data2}) #, crs='EPSG:4326)
    except:
        gdr = gpd.GeoDataFrame({'feature': [0], 'geometry': data2}) #, crs='EPSG:4326)
    gdr.to_file(f"{dir}/{foldername}/input/{foldername}.shp")    #------> This is the objective!!!!!! 
    ###################### Remove previous data!!! 
    f = []
    for (dirpath, dirnames, filenames) in os.walk(f"{dir}/{foldername}/output/"):
        f.extend(filenames)
        break
    for file in f:
        if re.search('_Perimeters',file) and len(file.split('_'))>2:
            tt=int(file.split('_')[1])
            st=int(file.split('_')[0])
            if st==prefix and st+tt>prefix+time:
                try:
                    out=os.system(f"rm {dir}/{foldername}/output/{file}")
                except:
                    print(f"remove failed")
        else:
            try:
               out=os.system(f"rm {dir}/{foldername}/output/{file}")
            except:
                print(f"remove failed")
    #################################################################

def CreateDyRxfire(data, foldername,dir,UID,wind=5,direction=270): # the location of the fireline, the length of the fireline and the number of the fires. 
    os.system(f"mkdir {dir}/{foldername}")
    os.system(f"cp -r {dir}/Template_burn/input {dir}/{foldername}")
    try:
        os.system(f"rm -r  {dir}/{foldername}/output")
        os.system(f"mkdir {dir}/{foldername}/output")
    except:
        os.system(f"mkdir {dir}/{foldername}/output")
    data.to_file(f"{dir}/{foldername}/input/seelin.shp")
    gap=20
    offset=50
    width=3
    SimTime=60
    if len(UID)==1:
        # Get Boundary: 
        newdata = gpd.GeoDataFrame()
        newdata['geometry'] = None
        poly=data.loc[UID[0],'geometry']
        poly=Polygon(poly)
        site=data.loc[UID[0],'geometry']
        x,y=site.coords.xy
        mx,my,bx,by=(min(x),min(y),max(x),max(y))
        x=mx+offset
        cout=0
        while x<bx-offset:
            line=[(int(x),int(my)-10),(int(x),int(by)+10)]
            if poly.intersection(LineString(line)).geom_type == 'MultiLineString':
                for linestr in poly.intersection(LineString(line)):
                    c=list(linestr.coords)
                    y=[i[1] for i in c]
                    if y[0]+offset<y[1]-offset:
                        coords=[(x,y[0]+offset), (x,y[1]-offset), (x+width,y[1]-offset), (x+width,y[0]+offset), (x, y[0]+offset)]
                        newdata.loc[cout, 'geometry'] = Polygon(coords)
                        cout=cout+1
                        #plt.plot([i[0] for i in coords], [i[1] for i in coords])
            else:
                c=list(poly.intersection(LineString(line)).coords)
                y=[i[1] for i in c]
                if y[0]+offset<y[1]-offset:
                    coords=[(x,y[0]+offset), (x,y[1]-offset), (x+width,y[1]-offset), (x+width,y[0]+offset), (x, y[0]+offset)]
                    newdata.loc[cout, 'geometry'] = Polygon(coords)
                    cout=cout+1
            x=x+gap
    ####################### Start_Sim, NewData is 
    time=0
    count=1  # how many fire stripes initially? 
    cudata=newdata[:count]#newdata.loc[0]
    burngap=10
    simdur=40
    cudata.to_file(f"{dir}/{foldername}/input/{foldername}.shp")
    while time<=SimTime and count<len(newdata):
        DyFarsitefile(foldername, dir,time,simdur,wind=wind,direction=direction)    #Run with current time with name is time
        ptime=time
        time=time+burngap+random.randint(0, 5)
        poly=newdata.loc[count,'geometry']
        Addfire(foldername, dir, ptime, time-ptime, poly)  # update the  f"{dir}/{foldername}/input/{foldername}.shp file. 
        count=count+1
    if time+simdur<SimTime:
        DyFarsitefile(foldername, dir,time+simdur,SimTime-time-simdur+60)    #Run with current time with name is time

def DyFarsitefile(foldername, dir,time,simdur,step=1,wind=5,direction=270):   
    #write testfile
    tmp=[0,0]
    simdur=simdur+20  #I do not why it always stop earlier! 
    if simdur>=60:
        tmp[0]=simdur//60
        tmp[1]=simdur%60
    else:
        tmp[1]=simdur
    #print(f"Sim time {tmp}")
    WriteInput(foldername,dir,dura=tmp, step=step,wind=wind,direction=direction)
    f = open(f"{dir}/{foldername}/{foldername}_TEST.txt", "w")
    f.write(f"{dir}/{foldername}/input/Burn.lcp ")# write landscape
    f.write(f"{dir}/{foldername}/input/Burn.input ") # write input file
    f.write(f"{dir}/{foldername}/input/{foldername}.shp ")# write ignition fire
    #f.write(f"{dir}/{foldername}/input/FQbarrier.shp ")
    f.write(f"{dir}/{foldername}/input/seelin.shp ")   # We can change the barrier!!! 
    f.write(f"{dir}/{foldername}/output/{time} 0")   # Output
    f.close()
    print(f"Command: {dir}/src/TestFARSITE {dir}/{foldername}/{foldername}_TEST.txt")
    try:
        out=os.system(f"{dir}/src/TestFARSITE {dir}/{foldername}/{foldername}_TEST.txt  >/dev/null 2>&1")
        #out=os.system(f"{dir}/src/TestFARSITE {dir}/{foldername}/{foldername}_TEST.txt")
        #print(f"see out", out.read())
        print(f"Generate the fire simulation successfully! Simulation from {time} duration  {simdur} ")
    except:
        print(f"Got error when simulating the fire spread")
  
def TestRunningtime(foldername, dir):
    Dis_Res=[1,2,3,4,5,10,20]
    Pre_Res=[1,3,5,10,20]
    Simtime=[1,2,3,4,5,6]
    Timestep=[1,2,3,4,5,6,10]
    Pars={'Dis':Dis_Res,'Pre':Pre_Res,'Sim':Simtime,'Tim':Timestep}
    logfile=open(f"{dir}/{foldername}_runningtime2.txt", "w")
    for key, values in Pars.items():
        for v in values:
            if key=='Dis':
                WriteInput(foldername,dir,dis_res=v)
            elif key=='Pre':
                WriteInput(foldername,dir,pre_res=v)
            elif key=='Sim':
                tmp=[0,0];v=v*60
                if v>=60:
                    tmp[0]=v//60
                    tmp[1]=v%60
                else:
                    tmp[1]=v
                WriteInput(foldername,dir,dura=tmp)
            elif key=='Tim':
                WriteInput(foldername,dir,step=v)
            outputfile=f"{foldername}_{key}_{v}"
            try:
                os.system(f"mkdir {dir}/{outputfile}")
            except:
                os.system(f"rm -r {dir}/{outputfile}")
                os.system(f"mkdir {dir}/{outputfile}")
            f = open(f"{dir}/{foldername}/{foldername}_TEST.txt", "w")
            f.write(f"{dir}/{foldername}/input/Burn.lcp ")# write landscape
            f.write(f"{dir}/{foldername}/input/Burn.input ") # write input file
            f.write(f"{dir}/{foldername}/input/{foldername}.shp ")# write ignition fire
            #f.write(f"{dir}/{foldername}/input/FQbarrier.shp ")
            f.write(f"{dir}/{foldername}/input/seelin.shp ")   # We can change the barrier!!! 
            f.write(f"{dir}/{outputfile}/runttime 0")   # Output
            f.close()  
            #try:
            cut=time.time()
            out=os.system(f"{dir}/src/TestFARSITE {dir}/{foldername}/{foldername}_TEST.txt >/dev/null 2>&1")
            runtime=time.time()-cut
            logfile.write(f"{key} {v} {runtime}\n") 
            print(f"Generate the fire simulation successfully! key {key} value {v} ")
            #except:
                #print(f"Got error when simulating the fire spread")
    logfile.close()
            
def Farsitefile(foldername, dir):   
    os.system(f"mkdir {dir}/{foldername}")
    os.system(f"cp -r {dir}/Template_burn/input {dir}/{foldername}")
    os.system(f"mkdir {dir}/{foldername}/output")
    #write testfile
    WriteInput(foldername,dir)
    f = open(f"{dir}/{foldername}/{foldername}_TEST.txt", "w")
    f.write(f"{dir}/{foldername}/input/Burn.lcp ")# write landscape
    f.write(f"{dir}/{foldername}/input/Burn.input ") # write input file
    f.write(f"{dir}/{foldername}/input/{foldername}.shp ")# write ignition fire
    #f.write(f"{dir}/{foldername}/input/FQbarrier.shp ")
    f.write(f"{dir}/{foldername}/input/seelin.shp ")   # We can change the barrier!!! 
    f.write(f"{dir}/{foldername}/output/{foldername} 0")
    f.close()
    try:
    # out=os.popen('pwd')
    # print(f"see out", out.read())
    # out.close()
        #print(f"command {dir}/src/TestFARSITE {dir}/{foldername}/{foldername}_TEST.txt ")
        out=os.system(f"{dir}/src/TestFARSITE {dir}/{foldername}/{foldername}_TEST.txt >/dev/null 2>&1")
    #os.system(f"{dir}/src/TestFARSITE {dir}/{foldername}/{foldername}_TEST.txt")
    #subprocess.run([f"{dir}/src/TestFARSITE", f"{dir}/{foldername}/{foldername}_TEST.txt"])
    # p = Popen(f"pwd",stdin=PIPE,stdout=PIPE,shell=True,close_fds=True,bufsize=132)
    #p = Popen(f"{dir}/src/TestFARSITE {dir}/{foldername}/{foldername}_TEST.txt",shell=False)
    # p.communicate()  
    # p.wait()
        print(f"Generate the fire simulation successfully!")
    except:
        print(f"Got error when simulating the fire spread")
    #shape=[] 


if __name__ == "__main__":
    dir='/home/fangqiliu/eclipse-workspace_Python/Drone_path/CoveragePathPlanning-master/farsite/Rxfire/Burn_1/output/Burn_1'
    igfile='/home/fangqiliu/eclipse-workspace_Python/Drone_path/CoveragePathPlanning-master/farsite/Rxfire/Burn_1/input/FQburn'
    dir='/home/fangqiliu/eclipse-workspace_Python/Drone_path/CoveragePathPlanning-master/farsite'
    map2=[]
    EFA=[]
    file='CARB_BurnUnits/CARB_BurnUnits.shp'
    data=gpd.read_file(f"{dir}/{file}")
    data=ClearBarrier(data)
    BunsiteBound=(702460.0,4309700.0,702860.0,4310200.0 )
    #data=CreateWildfie(BunsiteBound,1)
    #WriteInput('FQ_burn', dir)  # Just read somthing. 
    #CreateRxfire(data, BunsiteBound,'FQ_burn',dir, [2]) # Create the ignition file 
    #Farsitefile('FQ_burn',dir)
    CreateDyRxfire(data, BunsiteBound,'FQ_burn',dir, [2])
    #TestRunningtime('FQ_burn',GetFirSimdir)
    
########################################### Old code, for something else. 
def CreateRxfire(data, BunsiteBound, foldername,dir,UID): # the location of the fireline, the length of the fireline and the number of the fires. 
    gap=10
    offset=20
    width=3
    if len(UID)==1:
        # Get Boundary: 
        newdata = gpd.GeoDataFrame()
        newdata['geometry'] = None
        poly=data.loc[UID[0],'geometry']
        poly=Polygon(poly)
        site=data.loc[UID[0],'geometry']
        x,y=site.coords.xy
        mx,my,bx,by=(min(x),min(y),max(x),max(y))
        x=mx+offset
        cout=0
        while x<bx-offset:
            line=[(int(x),int(my)-10),(int(x),int(by)+10)]
            if poly.intersection(LineString(line)).geom_type == 'MultiLineString':
                for linestr in poly.intersection(LineString(line)):
                    c=list(linestr.coords)
                    y=[i[1] for i in c]
                    if y[0]+offset<y[1]-offset:
                        coords=[(x,y[0]+offset), (x,y[1]-offset), (x+width,y[1]-offset), (x+width,y[0]+offset), (x, y[0]+offset)]
                        newdata.loc[cout, 'geometry'] = Polygon(coords)
                        cout=cout+1
                        #plt.plot([i[0] for i in coords], [i[1] for i in coords])
            else:
                c=list(poly.intersection(LineString(line)).coords)
                y=[i[1] for i in c]
                if y[0]+offset<y[1]-offset:
                    coords=[(x,y[0]+offset), (x,y[1]-offset), (x+width,y[1]-offset), (x+width,y[0]+offset), (x, y[0]+offset)]
                    newdata.loc[cout, 'geometry'] = Polygon(coords)
                    cout=cout+1
                    #plt.plot([i[0] for i in coords], [i[1] for i in coords])
            x=x+gap
        newdata.to_file(f"{dir}/{foldername}/input/{foldername}.shp")
        # newdata.plot()
        # plt.show()
    
    # newdata.to_file(f"{dir}/{foldername}/input/{foldername}.shp")
    return newdata






def UpadteEFA(Pmap, area_map,time,EFA):
    if len(Pmap)==1:
        img=area_map
        EFA = np.full(img.shape, -1,dtype=np.uint8)
        EFA[img == 0] = time
        x,y= np.where(img==0)
    else:
        tmp=area_map-Pmap
        EFA[tmp==1]=time
        x,y=np.where(EFA==time)
    return EFA, x,y

def logEFA(EFAdict,Primdict, filename, shape, Res):
    f = open(filename, "w") 
    print(shape[0])
    f.write(f"{shape[0]} {shape[1]}\n ")
    for key in EFAdict.keys():
        row=np.array(EFAdict[key])
        f.write(f"{key} {len(list(row[0]))} ")
        for i in list(row[0]):
            f.write(f"{i},")
        f.write(f" ")
        for i in list(row[1]):
            f.write(f"{i},")
        f.write(f"\n")
        f.write(f"Prim {len(list(Primdict.get(key)[0]))} ")
        for j in list(Primdict.get(key)[0]):
            f.write(f"{j},")
        for j in list(Primdict.get(key)[1]):
            f.write(f"{j},")
        f.write(f"\n")
    f.close() 

def GetEFA(filename):
    lines = []
    with open(filename) as f:
        lines = f.readlines()  

def PutIginition(filename,dir):
    tofile=f"{dir}_0_Perimeters"
    c1=f"cp {filename}.shp {tofile}.shp"
    c2=f"cp {filename}.shx {tofile}.shx"
    os.system(c1)
    os.system(c2)
    print(f"Run {c1}")
    
def CreateCirecle(center, radius):
    def get_circle_coord(theta, x_center, y_center, radius):
        x = radius * math.cos(theta) + x_center
        y = radius * math.sin(theta) + y_center
        return (x,y)
    # This function gets all the pairs of coordinates
    def get_all_circle_coords(x_center, y_center, radius, n_points):
        thetas = [i/n_points * math.tau for i in range(n_points)]
        circle_coords = [get_circle_coord(theta, x_center, y_center, radius) for theta in thetas]
        return circle_coords
    x_c=center[0]
    y_c=center[1]
    circle_coords = get_all_circle_coords(x_c, y_c, radius, n_points = 20)
    poly=Polygon(circle_coords)

    return poly
def GetIgnitionCenter(BunsiteBound,num):
    sx, sy, bx, by =  BunsiteBound
    coors=[]
    for i in range(num):
        x=random.randint(sx, bx)
        y=random.randint(sy, by)
        coors.append([x,y])
    return coors
def GetIgnitionRadius(BunsiteBound,num):
    sx, sy, bx, by =  BunsiteBound
    rads=[]
    for i in range(num):
        r=random.randint(50,min(150,min((bx-sx),(by-sy))))
        rads.append(r)
    return rads


def GetInigitionNear(U_x,U_y,BunsiteBound):
    sx, sy, bx, by =  BunsiteBound
    #print(f"check U_x, U_y {U_x} {U_y}")
    U_x=max(sx,U_x)
    U_x=min(U_x,bx)
    U_y=max(sy,U_y)
    U_y=min(U_y,by)   
    #print(f"Why {bx} {U_x}")
    p_x=random.randint(10,min(100,(bx-U_x)))
    p_y=random.randint(10,min(100,(by-U_y)))
    return (U_x+p_x),(U_y+p_y)
    
def CreateWildfie(BunsiteBound,num):
    coors=GetIgnitionCenter(BunsiteBound,num)
    rads=GetIgnitionRadius(BunsiteBound,num)
    #print(rads)
    newdata = gpd.GeoDataFrame()
    newdata['geometry'] = None
    for i in range(num):
        poly=CreateCirecle(coors[i], rads[i])
        newdata.loc[i, 'geometry'] = poly
    newdata.to_file('see.shp')
    return newdata


