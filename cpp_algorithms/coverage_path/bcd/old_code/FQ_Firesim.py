
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
import json
import fiona
from shapely.geometry import shape 
import pandas as pd
def get_hv_wh(BunsiteBound):
    """
    Get haversine calcualted width and height of
    the smallest bounding rectangle of the coverage area.
    """
    llng, llat, rlng, rlat =  BunsiteBound
    w=abs(llng-rlng)
    h=abs(llat-rlat)
    print(f"FQ Weight: {w}; Height: {h}")
    return w, h
"""
Convert to raster
"""
def get_scale(BunsiteBound, meter=1):
    """
#     Returns the supposed longside of the area.
#     """
    w,h = get_hv_wh(BunsiteBound)
    return int(np.round((np.array([w,h])/meter).max()))

def imshow(area_map,r=1,c=1,i=1,figsize=(5,5),cmap="viridis"):
    """
    Display with no interpolation.
    """
    if r < 2 and c < 2 or i == 1:
        plt.figure(figsize=figsize)
    plt.subplot(r,c,i)
    ax = plt.imshow(area_map.T,interpolation='none',cmap=cmap,origin='lower')
    plt.axis('off');
    return ax

def get_raster(gpdf_final, scale=2000, BunsiteBound=0,CRS = f"EPSG:4326"):
    """
    Returns rasterised version for the projection
    """
    Res=10
    mn=np.array(BunsiteBound[:2])
    sh=(np.array(BunsiteBound[2:])-mn)/Res# FQ change the boundary
    sh=np.int64(sh)
    p = np.full(sh,-1) #FQ change that
    empty=np.full(sh,-1) #for record perimeter
    prim=[[]for i in range(2)]
    for i in range(len(gpdf_final)):
        tmp=gpdf_final
        try:
            shp = tmp.to_crs(crs=CRS)
        except:
            shp = tmp.set_crs(crs="EPSG:4326")
        ext = np.array(shp.geometry[i].exterior.coords).copy()
        mix = (ext - mn)/Res
        #mx = mix.max()
        mx=(np.array(BunsiteBound[2:])-mn).max()  #FQ changed that
        mix *= scale/mx
        mix = np.int64(mix)
        br,bc=mix.T # for the fire perimeter 
        empty[br, bc]=0
        prim[0]=prim[0]+list(br)
        prim[1]=prim[1]+list(bc)
        r,c = polygon(*mix.T,sh)   #fq? should reverse that? 
        p[r,c] = 0
        p[br,bc]=3
    x,y=np.where(empty==0)
    imshow(empty)
    imshow(p)
    plt.show()
    print(f"FQ change to  {len(x)}")
    return p,[x,y]
def Firefront(maps):
    cu_map=maps[0]
    fu_map=maps[1]
    return cu_map-fu_map

def draw(area_map_):
    img=area_map_
    matrix = np.full(img.shape, 0, dtype=np.uint8)
    matrix[img == -1] = 255
    imshow(matrix, figsize=(8, 8), cmap="cividis")   
    plt.show(block=True)

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
####################################### Rxfire
def CreateRectangle(x_dl,y_dl, weight,height):# gag 10 m
    x_r=x_dl+weight
    y_u=y_dl+height
    coords=[(x_dl,y_dl), (x_r,y_dl), (x_r,y_u), (x_dl,y_u), (x_dl, y_dl)]
    poly=Polygon(coords)
    return poly 
def GetInigitionNear(U_x,U_y,BunsiteBound):
    sx, sy, bx, by =  BunsiteBound
    print(f"check U_x, U_y {U_x} {U_y}")
    U_x=max(sx,U_x)
    U_x=min(U_x,bx)
    U_y=max(sy,U_y)
    U_y=min(U_y,by)   
    print(f"Why {bx} {U_x}")
    p_x=random.randint(10,min(100,(bx-U_x)))
    p_y=random.randint(10,min(100,(by-U_y)))
    return (U_x+p_x),(U_y+p_y)
    
def CreateWildfie(BunsiteBound,num):
    coors=GetIgnitionCenter(BunsiteBound,num)
    rads=GetIgnitionRadius(BunsiteBound,num)
    print(rads)
    newdata = gpd.GeoDataFrame()
    newdata['geometry'] = None
    for i in range(num):
        poly=CreateCirecle(coors[i], rads[i])
        newdata.loc[i, 'geometry'] = poly
    newdata.to_file('see.shp')
    return newdata

def SeePolygon(data):
    polygons=[]
    for index, row in data.iterrows():
        poly = row['geometry']
        print(f"seeeee {poly}")
        polygons.append(poly)
    print(f"whyyyyy {polygons[0]}")
    u=cascaded_union([polygons[0],polygons[1],polygons[2]])
    u2=cascaded_union([polygons[3],polygons[6]])
    x,y=u2.exterior.coords.xy
    newdata = gpd.GeoDataFrame()
    newdata['geometry'] = None
    newdata.loc[0, 'geometry'] = u2.boundary[0]
    newdata.loc[1, 'geometry'] = polygons[4].boundary
    newdata.loc[2, 'geometry'] = u.boundary
    foldername='FQ_burn'
    newdata.to_file(f"{dir}/{foldername}/input/seelin.shp")
    da2=gpd.read_file(f"{dir}/{foldername}/input/seelin.shp")
    return da2
    # da2.plot()
    # plt.show()  
    
def CreateRxfire(data, BunsiteBound, foldername,dir,UID): # the location of the fireline, the length of the fireline and the number of the fires. 
    gap=10
    offset=20
    width=1
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
        #print(f"see Minimum, Maximum", mx,my,bx,by)
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
    Farsitefile(foldername,dir)
    # newdata.to_file(f"{dir}/{foldername}/input/{foldername}.shp")
    return newdata

def WriteInput(foldername, dir,step=1,initime=(0,0), dura=(0,40)): # The simulation time 100 =1:00) duration<24 hours!
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
            #print(f"wind", wind, winddric)
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
        else:
            fin.write(line)


def CreateDyRxfire(data, BunsiteBound, foldername,dir,UID): # the location of the fireline, the length of the fireline and the number of the fires. 
    gap=20
    offset=40
    width=1
    SimTime=3*60
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
        #print(f"see Minimum, Maximum", mx,my,bx,by)
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
    ####################### Start_Sim, NewData is 
    #print(f"see newdata", newdata[:1])
    time=0
    count=3
    cudata=newdata[:count]#newdata.loc[0]
    burngap=10
    simdur=40
    print(f"see cur {cudata}")
    cudata.to_file(f"{dir}/{foldername}/input/{foldername}.shp")
    while time<=SimTime and count<len(newdata):
        DyFarsitefile(foldername, dir,time,simdur)    #Run with current time with name is time
        print(f"see time {time}  ")
        ptime=time
        time=time+burngap+random.randint(0, 5)
        print(f"see time {time} {ptime} ")
        poly=newdata.loc[count,'geometry']
        Addfire(foldername, dir, ptime, time-ptime, poly)
        count=count+1
        
    #############Check! 
    if time+simdur<SimTime:
        DyFarsitefile(foldername, dir,time,SimTime-time-simdur+30)    #Run with current time with name is time
        
    #Farsitefile(foldername,dir)
    # newdata.to_file(f"{dir}/{foldername}/input/{foldername}.shp")
    #return newdata

def Addfire(foldername, dir, prefix, time, poly):   # Update the ignition file! because we add some fire!!!! 
    Contine=True
    while True and Contine:
        try:
            collection = list(fiona.open(f"{dir}{foldername}/output/{prefix}_{time}_Perimeters.shp",'r'))
            df1 = pd.DataFrame(collection)
            #Check Geometry
            def isvalid(geom):
                try:
                    shape(geom)
                    return 1
                except:
                    return 0
            df1['isvalid'] = df1['geometry'].apply(lambda x: isvalid(x))
            df1 = df1[df1['isvalid'] == 1]
            collection = json.loads(df1.to_json(orient='records'))
            #Convert to geodataframe
            data = gpd.GeoDataFrame.from_features(collection)
            #data=gpd.read_file(f"{dir}{foldername}/output/{prefix}_{time}_Perimeters.shp")
            break 
        except: 
            print(f"find {dir}{foldername}/output/{prefix}_{time}_Perimeters.shp")
            sleep(0.5)
            # f = []
            # for (dirpath, dirnames, filenames) in os.walk(f"{dir}{foldername}/output/"):
            #     f.extend(filenames)
            #     break
            # print(f"find {dir}{foldername}/output/{prefix}_{time}_Perimeters.shp")
            # print(f"see files {len(f)} ")
            # for fn in f:
            #     if re.match(f"{prefix}_{time}_Perimeters.shp",fn):
            #         print(f"find it!!!!! {fn}")
            #         collection = list(fiona.open(f"{dir}{foldername}/output/{prefix}_{time}_Perimeters.shp",'r'))
            #         df1 = pd.DataFrame(collection)
            #         #Check Geometry
            #         def isvalid(geom):
            #             try:
            #                 shape(geom)
            #                 return 1
            #             except:
            #                 return 0
            #         df1['isvalid'] = df1['geometry'].apply(lambda x: isvalid(x))
            #         df1 = df1[df1['isvalid'] == 1]
            #         collection = json.loads(df1.to_json(orient='records'))
            #         #Convert to geodataframe
            #         data = gpd.GeoDataFrame.from_features(collection)
            #         #data=gpd.read_file(f"{dir}{foldername}/output/{prefix}_{time}_Perimeters.shp")
            #         Contine=False
            #         break 
    print(f"change: {dir}{foldername}/output/{prefix}_{time}_Perimeters.shp")
    try:
        print(f"{[data.loc[i, 'geometry'] for i in range(len(data))]}")
        geoms=[data.loc[i, 'geometry'] for i in range(len(data))]+[poly]
        data2=shapely.ops.unary_union([geom if geom.is_valid else geom.buffer(0) for geom in geoms]) 
    except:
        data.plot()
        x,y=poly.exterior.coords.xy
        plt.plot(x,y,color='red')
        plt.show()
        #data2=cascaded_union([data.loc[i, 'geometry'] for i in range(len(data))])
        geoms=[data.loc[i, 'geometry'] for i in range(len(data))]
        data2=shapely.ops.unary_union([geom if geom.is_valid else geom.buffer(0) for geom in geoms])
    features = [i for i in range(len(data2))]
    gdr = gpd.GeoDataFrame({'feature': features, 'geometry': data2}) #, crs='EPSG:4326)
    gdr.to_file(f"{dir}/{foldername}/input/{foldername}.shp")    #------> This is the objective!!!!!! 
    ###################### Remove previous data!!! 
    f = []
    for (dirpath, dirnames, filenames) in os.walk(f"{dir}{foldername}/output/"):
        f.extend(filenames)
        break
    for file in f:
        if re.search('_Perimeters',file) and len(file.split('_'))>2:
            tt=int(file.split('_')[1])
            st=int(file.split('_')[0])
            if st==prefix and st+tt>prefix+time:
                try:
                    out=os.popen(f"rm {dir}{foldername}/output/{file}")
                except:
                    print(f"remove failed")
        else:
            try:
               out=os.popen(f"rm {dir}{foldername}/output/{file}")
            except:
                print(f"remove failed")
    #################################################################

def DyFarsitefile(foldername, dir,time,simdur):   
    os.system(f"mkdir {dir}/{foldername}")
    os.system(f"cp -r {dir}/Template_burn/input {dir}/{foldername}")
    os.system(f"mkdir {dir}/{foldername}/output")
    #write testfile
    tmp=[0,0]
    if simdur>60:
        tmp[0]=simdur/60
        tmp[1]=simdur%60
    else:
        tmp[1]=simdur
    WriteInput(foldername,dir,dura=tmp)
    f = open(f"{dir}/{foldername}/{foldername}_TEST.txt", "w")
    f.write(f"{dir}/{foldername}/input/Burn.lcp ")# write landscape
    f.write(f"{dir}/{foldername}/input/Burn.input ") # write input file
    f.write(f"{dir}/{foldername}/input/{foldername}.shp ")# write ignition fire
    #f.write(f"{dir}/{foldername}/input/FQbarrier.shp ")
    f.write(f"{dir}/{foldername}/input/seelin.shp ")   # We can change the barrier!!! 
    f.write(f"{dir}/{foldername}/output/{time} 0")   # Output
    try:
        out=os.popen(f"{dir}/src/TestFARSITE {dir}/{foldername}/{foldername}_TEST.txt")
        print(f"Generate the fire simulation successfully!")
    except:
        print(f"Got error when simulating the fire spread")
        





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
    try:
        out=os.popen(f"{dir}/src/TestFARSITE {dir}/{foldername}/{foldername}_TEST.txt")
        # preprocessed = process.read()
        #process.close()
        #p = Popen(f"{dir}/src/TestFARSITE {dir}/{foldername}/{foldername}_TEST.txt",stdin=PIPE,stdout=PIPE,shell=True,close_fds=True,bufsize=232)
        #out=proc.stdout.readline()
        #p = Popen(f"ls",stdin=PIPE,stdout=PIPE,shell=True,close_fds=True,bufsize=132)
        #c=p.stdout.read()
        #print(c)
        print(f"Generate the fire simulation successfully!")
    except:
        print(f"Got error when simulating the fire spread")
    #shape=[] 




if __name__ == "__main__":


    dir='/home/fangqiliu/eclipse-workspace_Python/Drone_path/CoveragePathPlanning-master/farsite/Rxfire/Burn_1/output/Burn_1'
    igfile='/home/fangqiliu/eclipse-workspace_Python/Drone_path/CoveragePathPlanning-master/farsite/Rxfire/Burn_1/input/FQburn'
    dir='/home/fangqiliu/eclipse-workspace_Python/Drone_path/CoveragePathPlanning-master/farsite/'
    map2=[]
    EFA=[]
    
    file='CARB_BurnUnits/CARB_BurnUnits.shp'
    data=gpd.read_file(f"{dir}{file}")
    data=SeePolygon(data)
    # data.plot()
    # plt.show()
    #BunsiteBound=(701235.0,4308525.0,704355.0,4311675.0)   
    BunsiteBound=(702460.0,4309700.0,702860.0,4310200.0 )
    #data=CreateWildfie(BunsiteBound,1)
    WriteInput('FQ_burn', dir)  # Just read somthing. 
    #CreateRxfire(data, BunsiteBound,'FQ_burn',dir, [2])
    CreateDyRxfire(data, BunsiteBound,'FQ_burn',dir, [2])
    #DyFarsitefile('FQ_burn',dir)
    #PutIginition(igfile,dir)
    



# def CreateRxfire_Old(data, BunsiteBound, foldername,dir,UID): # the location of the fireline, the length of the fireline and the number of the fires. 
#     GPS="EPSG:4326"
#     ZONE10="EPSG:32610"
#     UA=(38.91922,-120.65556)
#     UB=(38.91706,-120.65802)
#     UC=(38.91546,-120.66233)
#     gap=10
#     Unit=[UA,UB,UC]
#     Unit=[UC]
#     transformer = Transformer.from_crs(GPS, "EPSG:32610")
#     newdata = gpd.GeoDataFrame()
#     newdata['geometry'] = None
#     c=0
#     fn=[5,3,7]
#     for i in range(len(Unit)):
#         print(Unit[i][0],Unit[i][1])
#         e,n=transformer.transform(Unit[i][0],Unit[i][1])
#         print(f"check {e} {n}")
#         x,y=GetInigitionNear(int(e),int(n),BunsiteBound)
#         for j in range(fn[i]):
#             x=x+gap*j
#             poly=CreateRectangle(x,y,3,20)
#             newdata.loc[c, 'geometry'] = poly
#             c=c+1
#     Farsitefile(foldername,dir)
#     newdata.to_file(f"{dir}/{foldername}/input/{foldername}.shp")
#     return newdata















