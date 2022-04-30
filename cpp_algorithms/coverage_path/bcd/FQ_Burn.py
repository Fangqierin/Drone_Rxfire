import math
import time
import numpy as np
from tqdm.auto import tqdm
from warnings import warn
from bcd_helper import backtracking_list
import sys
#sys.path.append('../../../')
from bcd_helper import imshow, imshow_scatter
import matplotlib.pyplot as plt
import geopandas as gpd
from skimage.draw import polygon
from shapely.geometry import Point, LineString, Polygon
from shapely.ops import cascaded_union
from descartes import PolygonPatch
from networkx.algorithms.distance_measures import diameter
from shapely.wkt import loads
#from common_helpers import get_all_area_maps, get_random_coords, get_area_map

def visit(matrix, x, y, memory,obstacle):
    matrix[(x,y)] = obstacle[0] # 150 == visited
    memory.append((x, y))
    return x,y
def is_bounded(coord, shape):
    """
    Checks if a coord (x,y) is within bounds.
    """
    x,y = coord
    g,h = shape
    lesser = x < 0 or y < 0
    greater = x >= g or y >= h
    if lesser or greater:
        return False
    return True
def is_valid(coord, area_map, obstacle = -1):
    """
    Check is a coord (x,y) is bounded and not
    on an obstacle.
    """
    coord = tuple(coord)
    is_b = is_bounded(coord, area_map.shape)
    if is_b:
        is_on_obs = False
        if isinstance(obstacle, list):
            for obs in obstacle:
                is_on_obs |= area_map[coord] == obs
        else:
            is_on_obs  = area_map[coord] == obstacle
        if not is_on_obs:
            return True
    return False

def boustrophedon(matrix, diameter, x, y, memory,direction, obstacle):
    # TODO :: Variable diameter support
    udlr = {
        "u": lambda x,y : (x-diameter,y),
        "d": lambda x,y : (x+diameter,y),  # FQ comment 
        "l": lambda x,y : (x,y-diameter),
        "r": lambda x,y : (x,y+diameter),
    }
    udlr = {
        "u": lambda x,y : (x,y+diameter),
        "d": lambda x,y : (x,y-diameter),  # FQ comment 
        "l": lambda x,y : (x-diameter,y),
        "r": lambda x,y : (x+diameter,y),
    }
    u = "u";d = "d";r = "r";l = "l"
    visit(matrix, x, y, memory,obstacle)
    while True:
        if direction=='UP':
            dir_ = [u,d,r,l]
            fdir=l
        else:
            dir_ = [r,l,u,d]
            fdir=d
            #dir_ = [u,d,r,l]
        while len(dir_) > 0:
            d_ = dir_.pop(0)   
            x_, y_ = udlr[d_](x,y)
            if is_valid((x_,y_), matrix, obstacle):
                x, y = visit(matrix, x_, y_, memory,obstacle)
                break
            elif d_ == fdir:    # FQ comment 
                return x, y
            
def Redirect(matrix,cover,start_point,obstacle):
    x=[];y=[]
    for i in cover:
        x1,y1=np.where(matrix==i)
        x=x+list(x1)
        y=y+list(y1)
    x=np.array(x)
    y=np.array(y)
    xi=min(x)
    xx=max(x)
    #print(f"see {cover} {x,y}")
    if abs(start_point[0]-xi)<abs(start_point[0]-xx):
        bx1=xi
    else:
        bx1=xx
    xl=list(np.where(x==bx1))
    yl=min(list(y[k] for k in xl))
    yi=min(yl)
    yx=max(yl)
    if abs(start_point[1]-yi)<abs(start_point[1]-yx):
        by1=yi
    else:
        by1=yx
    ##############################################################  
    # if abs(start_point[0]-min(y))<abs(start_point[0]-max(y)):
    #     by2=min(y)
    # else:
    #     by2=max(y)
    # yl=list(np.where(y==by2))
    # xl=min(list(x[k] for k in yl))
    # if abs(start_point[1]-min(xl))<abs(start_point[1]-max(xl)):
    #     bx2=min(xl)
    # else:
    #     bx2=max(xl)
    bx=bx1
    by=by1
    # if (start_point[0]-bx1)**2+(start_point[1]-by1)**2 <(start_point[0]-bx2)**2+(start_point[1]-by2)**2:
    #     bx=bx1;by=by1
    # else:
    #     bx=bx2;by=by2
    print(f" redirect to {bx}, {by}")
    return int(bx), int(by)

def Planner(matrix, start_point, break_after_bous=False, timeit=True, direction='UP', cover=0, obstacle=[0]):
    width, height = matrix.shape
    radius=0.5 # FQ changed this
    diameter=int(2*radius) 
    x, y = start_point
    memory=[]
    if not is_valid((x,y), matrix, obstacle):
        print(f" Initial location is at obstacle {obstacle}!")
        bx,by=Redirect(matrix, cover,start_point,obstacle)
        x=bx; y=by
        visit(matrix, x, y, memory,obstacle)
        visit(matrix, bx, by, memory,obstacle)
    critical=[]
    cri_end=[]
    while True:
        critical_x, critical_y = boustrophedon(matrix, diameter, x, y, memory, direction, obstacle)
        critical.append([critical_x,critical_y])
        if break_after_bous:
            print("break point",critical_x,critical_y)
            break
        next_, is_end = backtracking_list(memory, diameter, matrix, critical_x, critical_y,obstacle)
        if is_end:
            x=[];y=[]
            for i in cover:
                x1,y1=np.where(matrix==i)
                x=x+list(x1)
                y=y+list(y1)
            if len(x)>0:
                x=np.array(x)
                y=np.array(y)
                x,y=Redirect(matrix, cover,start_point,obstacle)
                cri_end.append([x,y])
                print(f" jump to another area!")
            else:
                break
        else:
            x,y = next_
            memory.append(next_)
            cri_end.append([x,y])
    #times_avg = timeit and {k+"_avg":np.array(times[k]).mean() for k in times}
    #times_tot = timeit and {k+"_tot":np.array(times[k]).sum() for k in times}
    #times = timeit and {**times_avg, **times_tot}
    #end_time = timeit and (time.time() - start_time)
    #if timeit: times['total'] = end_time
    #timeit and printer(times)
    return memory, critical, cri_end
def get_hv_wh(BunsiteBound):
    """
    Get haversine calcualted width and height of
    the smallest bounding rectangle of the coverage area.
    """
    llng, llat, rlng, rlat =  BunsiteBound
    w=abs(llng-rlng)
    h=abs(llat-rlat)
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

def get_raster(gpdf_final, scale=2000, BunsiteBound=0,CRS = f"EPSG:4326",NF=-1):
    """
    Returns rasterised version for the projection
    """
    #assert len(gpdf_final) == 1
    perimeter=[]
    mn=np.array(BunsiteBound[:2])
    sh=np.array(BunsiteBound[2:])-mn # FQ change the boundary
    sh=np.int64(sh)
    p = np.full(sh,NF) #FQ change that
    for i in range(len(gpdf_final)):
        shp=gpdf_final.loc[i, 'geometry']
        ext = np.array(shp.exterior)
        mix = ext - mn
        mx = mix.max()
        mx=(np.array(BunsiteBound[2:])-mn).max()  #FQ changed that
        mix *= scale/mx
        mix = np.int64(mix)
        r,c = polygon(*mix.T,sh)
        p[r,c] = i
        perimeter.append(mix)
    return p, perimeter


def get_vague_raster(gpdf_final, scale=2000, BunsiteBound=0,CRS = f"EPSG:4326",NF=-1,diameter=30,missions=[0]):
    """
    Returns rasterised version for the projection
    """
    #assert len(gpdf_final) == 1
    perimeter=[]
    mn=np.array(BunsiteBound[:2])
    sh=np.array(BunsiteBound[2:])-mn # FQ change the boundary
    sh=np.int64(sh/diameter)
    p = np.full(sh,NF) #FQ change that
    if -1 in missions:
        for i in range(len(gpdf_final)):
            shp=gpdf_final.loc[i, 'geometry']
            i=-2
            ext = np.array(shp.exterior)
            mix = ext - mn
            mx=(np.array(BunsiteBound[2:])-mn).max()  #FQ changed that
            mix1=np.ceil(mix/diameter)#*30
            mix1=np.int64(mix1)
            r,c = polygon(*mix1.T,sh)
            p[r,c] = i
            r,c = mix1.T
            p[r,c] = i
            mix2=np.floor(mix/diameter)#*30
            r,c = polygon(*mix2.T,sh)
            p[r,c] = i
            mix2=np.int64(mix2)
            r,c = mix2.T
            p[r,c] = i 
        if len(missions)==1:
            return p, []
        else:
            missions.remove(-1)
    for i in missions:
        shp=gpdf_final.loc[i, 'geometry']
        ext = np.array(shp.exterior)
        mix = ext - mn
        mx = mix.max()
        mx=(np.array(BunsiteBound[2:])-mn).max()  #FQ changed that
        mix1=np.ceil(mix/diameter)#*30
        r1,c1 = polygon(*mix1.T,sh)
        print(f"see shape {sh}")
        mix1=np.int64(mix1)
        p[r1,c1] = i
        r,c = mix1.T
        p[r,c] = i
        mix2=np.floor(mix/diameter)#*30
        r,c = polygon(*mix2.T,sh)
        p[r,c] = i
        mix2=np.int64(mix2)
        r,c = mix2.T
        p[r,c] = i
        perimeter.append(mix2)
    return p, perimeter
def GetBarrier(data):
    polygons=[]
    for index, row in data.iterrows():
        poly = row['geometry']
        polygons.append(poly)
    u=cascaded_union(polygons)
    newdata = gpd.GeoDataFrame()
    newdata['geometry'] = None
    newdata.loc[0, 'geometry'] = u.boundary
    newdata.loc[1, 'geometry'] = polygons[4].boundary
    foldername='FQ_burn'
    newdata.to_file(f"{dir}/{foldername}/input/seelin.shp")
    da2=gpd.read_file(f"{dir}/{foldername}/input/seelin.shp")
def DroneCPP(data,BunsiteBound, Mission, AllM, diameter, start_point, direction='UP'):
        for i in Mission:
            AllM.remove(i)
        scale = get_scale(BunsiteBound, meter=1)
        area_map_old, fire = get_raster(data, scale, BunsiteBound,NF=-1)
        area_map, f = get_vague_raster(data, scale, BunsiteBound,NF=-1,missions=Mission,diameter=diameter)
        img=area_map
        coverage_path,critical,cri_end = Planner(area_map,start_point,direction=direction,cover=Mission, obstacle=AllM)
        if len(coverage_path)>0:
            end_point = coverage_path[-1]
            x,y = np.array(coverage_path).T
            #Display by step 
            # for i in range(int(len(x)/40)):
            #       imshow(area_map_old, figsize=(8, 8), cmap="cividis")   
            #       imshow_scatter([start_point],color="lightgreen")
            #       imshow_scatter([end_point],color="red")
            #       # imshow_scatter(critical, alpha=0.4, color="red",s=20)
            #       # imshow_scatter(cri_end, alpha=0.4, color="black",s=50)
            #       imshow_scatter(coverage_path, alpha=0.4, color="black",s=5)
            #       plt.plot(x[0:i*40+2],y[0:i*40+2])
            #       plt.show()
            #imshow(area_map_old, figsize=(8, 8), cmap="cividis")   
            imshow(area_map, figsize=(8, 8), cmap="cividis")  # if you want drone's perspective
            #imshow_scatter(critical*30, alpha=0.4, color="red",s=20)
            # for k in Mission:
            #     if k!=-1:
            #         plt.plot([i[0] for i in fire[k]],[i[1] for i in fire[k]],'red')
            #imshow_scatter(cri_end, alpha=0.4, color="black",s=50)
            #plt.plot(x*diameter,y*diameter)
            plt.plot(x,y)  # if you want drone's perspective
            plt.show()
if __name__ == "__main__":
    start_point = (0, 0)
    #start_point = (1484, 2226)
    dir='/home/fangqiliu/eclipse-workspace_Python/Drone_path/CoveragePathPlanning-master/farsite/'
    file='CARB_BurnUnits/CARB_BurnUnits.shp'
    data=gpd.read_file(f"{dir}{file}")
    # ############################################### Drone do coverage
    BunsiteBound=(701235.0,4308525.0,704355.0,4311675.0)
    AllM=list(range(len(data)))+[-1,-2]
    Mission=[-1]
    diameter=50
    DroneCPP(data,BunsiteBound, Mission, AllM, diameter, start_point, direction='UP')
    ######################################### 