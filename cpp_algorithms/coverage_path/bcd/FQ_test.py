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
    x,y=np.where(matrix==cover)
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
    bx=bx1;by=by1
    # if (start_point[0]-bx1)**2+(start_point[1]-by1)**2 <(start_point[0]-bx2)**2+(start_point[1]-by2)**2:
    #     bx=bx1;by=by1
    # else:
    #     bx=bx2;by=by2
    print(f" redirect to {bx}, {by}")
    return bx1, by1

def Planner(matrix, start_point, break_after_bous=False, timeit=True, direction='UP', cover=0, obstacle=[0]):
    width, height = matrix.shape
    radius=20 # FQ changed this
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
        #print(f"see critical {critical_x,critical_y}")
        critical.append([critical_x,critical_y])
        # imshow(matrix, figsize=(8, 8), cmap="cividis")   
        # imshow_scatter([start_point],color="lightgreen")
        # imshow_scatter(critical, alpha=0.4, color="red",s=20)
        # imshow_scatter(memory, alpha=0.4, color="black",s=5)
        # plt.plot([memory[i][0]for i in range(len(memory))],[memory[i][1]for i in range(len(memory))])
        # plt.show()
        if break_after_bous:
            print("break point",critical_x,critical_y)
            break
        next_, is_end = backtracking_list(memory, diameter, matrix, critical_x, critical_y,obstacle)
        if is_end:
            break
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

def get_raster(gpdf_final, scale=2000, BunsiteBound=0,CRS = f"EPSG:4326",NF=-1,F=0):
    """
    Returns rasterised version for the projection
    """
    assert len(gpdf_final) == 1
    try:
        shp = gpdf_final.to_crs(crs=CRS)
    except:
        shp = gpdf_final.set_crs(crs=CRS)
    ext = np.array(shp.geometry[0].exterior).copy()
    ite=np.array(shp.geometry[0].interiors).copy()
    mn=np.array(BunsiteBound[:2])
    mix = ext - mn
    mx = mix.max()
    mx=(np.array(BunsiteBound[2:])-mn).max()  #FQ changed that
    mix *= scale/mx
    mix = np.int64(mix)
    sh=np.array(BunsiteBound[2:])-mn # FQ change the boundary
    sh=np.int64(sh)
    r,c = polygon(*mix.T,sh)
    p = np.full(sh,NF) #FQ change that
    p[r,c] = F
    return p, mix

if __name__ == "__main__":
    start_point = (0, 0)
    #start_point = (1484, 2226)
    file="try.shp"
    file='9_Perimeters.shp'
    data=gpd.read_file(file)
    for index, row in data.iterrows():
        poly = row['geometry']
    BunsiteBound=(-2096325.0,2037105.0,-2092695.0,2040735.0)
    scale = get_scale(BunsiteBound, meter=1)
    area_map, fire = get_raster(data, scale, BunsiteBound,NF=-1,F=0)
    img=area_map
    matrix = np.full(img.shape, 0, dtype=np.uint8)
    matrix[img == -1] = 255 
    AllM=[0,1,2,3,4,255]
    Mission=255
    AllM.remove(Mission)
    coverage_path,critical,cri_end = Planner(matrix,start_point,direction='UP',cover=Mission, obstacle=AllM)
    if len(coverage_path)>0:
        end_point = coverage_path[-1]
        x,y = np.array(coverage_path).T
        #Display 
        # for i in range(int(len(x)/400)):
        #      imshow(matrix, figsize=(8, 8), cmap="cividis")   
        #      imshow_scatter([start_point],color="lightgreen")
        #      imshow_scatter([end_point],color="red")
        #      # imshow_scatter(critical, alpha=0.4, color="red",s=20)
        #      # imshow_scatter(cri_end, alpha=0.4, color="black",s=50)
        #      imshow_scatter(coverage_path, alpha=0.4, color="black",s=5)
        #      plt.plot(x[0:i*400+2],y[0:i*400+2])
        #      plt.show()
        imshow(matrix, figsize=(8, 8), cmap="cividis")   
        imshow_scatter(critical, alpha=0.4, color="red",s=20)
        imshow_scatter(cri_end, alpha=0.4, color="black",s=50)
        plt.plot(x,y)
        plt.show()
        
        
        
        
        