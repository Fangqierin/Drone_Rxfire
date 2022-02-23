import math
import time
import numpy as np
from tqdm.auto import tqdm
from warnings import warn
from bcd_helper import backtracking_list, move_like_oxen, bcd_preprocess
import sys
#sys.path.append('../../../')
from common_helpers import is_valid, adjacency_test
#from cpp_algorithms.testers.display_funcs import printer
#from cpp_algorithms.testers.metrics import coverage_metrics
from common_helpers import imshow, imshow_scatter
import matplotlib.pyplot as plt
import geopandas as gpd
from skimage.draw import polygon

from common_helpers import get_all_area_maps, get_random_coords, get_area_map

def visit(matrix, x, y, memory):
    matrix[(x,y)] = 150 # 150 == visited
    memory.append((x, y))
    return x,y

def boustrophedon(matrix, diameter, x, y, memory):
    # TODO :: Variable diameter support
    udlr = {
        "u": lambda x,y : (x-diameter,y),
        "d": lambda x,y : (x+diameter,y),  # FQ comment 
        "l": lambda x,y : (x,y-diameter),
        "r": lambda x,y : (x,y+diameter),
    }
    u = "u";d = "d";r = "r";l = "l"
    visit(matrix, x, y, memory)
    
    while True:
        #print(f"why cannot stop?")
        dir_ = [u,d,r,l]
        #dir_ = [r,l,u,d]
        while len(dir_) > 0:
            d_ = dir_.pop(0)   
            x_, y_ = udlr[d_](x,y)
            if is_valid((x_,y_), matrix, [0, 150]):
                x, y = visit(matrix, x_, y_, memory)
                break
            elif d_ == l:    # FQ comment 
            #elif d_ == d:
                return x, y
def heuristic(start, goal):
    #Use Chebyshev distance heuristic if we can move one square either
    #adjacent or diagonal
    D = 1
    D2 = 1
    dx = abs(start[0] - goal[0])
    dy = abs(start[1] - goal[1])
    return D * (dx + dy) + (D2 - 2 * D) * min(dx, dy)
def get_vertex_neighbours(pos, diameter, width, height):
    n = []
    #Moves allow link a chess king
    for dx, dy in [(diameter,0),(-diameter,0),(0,diameter),(0,-diameter)]:
        x2 = pos[0] + dx
        y2 = pos[1] + dy
        if x2 < 0 or x2 > width-1  or y2 < 0 or y2 > height-1:
            continue
        n.append((x2, y2))
    return n
def bous_preprocess(area_map):
    """
    Returns matrix that in the form
    that is read by the main algorithm.
    """
    matrix = np.full(area_map.shape, 0, dtype=np.uint8)
    matrix[area_map == 0] = 255
    #print(f"FQ area_map{area_map}   matrix  {matrix}")
    return matrix

def AstarPath(M, Memory, path):
    for q in path:
        x=q[0]
        y=q[1]
        Memory.append((x, y))
def do_everything(matrix, start_point, break_after_bous=False, timeit=True):
    t = lambda :timeit and time.time()
    times = {
        "bous": [],
        "btrack":[],
        "astar":[]
    }
    store_time = lambda name,value: timeit and times[name].append(value)
    start_time = t()
    
    width, height = matrix.shape
    #radius=0.5
    radius=30 # FQ changed this
    diameter=int(2*radius) 
    x, y = start_point
    memory=[]
    backtrack_counts = 0
    point_find_failed = 0 
    critical=[]
    cri_end=[]
    while True:
        sw = t()
        critical_x, critical_y = boustrophedon(matrix, diameter, x, y, memory)
        store_time("bous", t() - sw)
        #print("break point",critical_x,critical_y)
        critical.append([critical_x,critical_y])
        #print("break point",critical_x,critical_y)
        if break_after_bous:
            print("break point",critical_x,critical_y)
            break
            
        sw = t()
        next_, is_end = backtracking_list(memory, diameter, matrix, critical_x, critical_y)
        #print(f"FQ next_ {next_}")
        x,y = next_
        memory.append(next_)
        cri_end.append([x,y])
        store_time("btrack", t() - sw)
        if is_end:
            break
        else:
            store_time("astar", t() - sw)
    times_avg = timeit and {k+"_avg":np.array(times[k]).mean() for k in times}
    times_tot = timeit and {k+"_tot":np.array(times[k]).sum() for k in times}
    times = timeit and {**times_avg, **times_tot}
    end_time = timeit and (time.time() - start_time)
    if timeit: times['total'] = end_time
    #timeit and printer(times)
    return memory, critical, cri_end
def get_hv_wh(BunsiteBound):
    """
    Get haversine calcualted width and height of
    the smallest bounding rectangle of the coverage area.
    """
    llng, llat, rlng, rlat =  BunsiteBound
    #print(f"FQ bounds", llng, llat, rlng, rlat)
#     ll = (llat,llng)
#     lr = (llat,rlng)
#     tr = (rlat,rlng)
#     tl = (rlat,llng)
    w=abs(llng-rlng)
    h=abs(llat-rlat)
    #w = distance.distance(ll,lr)
    #h = distance.distance(ll,tl)
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

def get_raster(gpdf_final, scale=2000, BunsiteBound=0,CRS = f"EPSG:4326"):
    """
    Returns rasterised version for the projection
    """
    assert len(gpdf_final) == 1
    try:
        shp = gpdf_final.to_crs(crs=CRS)
    except:
        shp = gpdf_final.set_crs(crs="EPSG:4326")
    
    ext = np.array(shp.geometry[0].exterior).copy()
    ite = map(np.array,shp.geometry[0].interiors)
    mn=np.array(BunsiteBound[:2])
    print(f"FQ mn {mn}")
    mix = ext - mn
    print(f"FQ mix {mix}")
    mx = mix.max()
    mx=(np.array(BunsiteBound[2:])-mn).max()  #FQ changed that
    print(f"FQ mx {mx}")
    mix *= scale/mx
    print(f"FQ mix {mix}")
    mix = np.int64(mix)
    sh=np.array(BunsiteBound[2:])-mn # FQ change the boundary
    sh=np.int64(sh)
    print(f"FQ sh {sh}")
    
    r,c = polygon(*mix.T,sh)
    p = np.full(sh,-1) #FQ change that

    p[r,c] = 0 
    
    for o in ite:
        r,c = polygon(*np.int16((o-mn)*scale/mx).T,sh)
        p[r,c] = -1
        
    return p, mn, mx, sh
if __name__ == "__main__":
    start_point = (0, 0)
    file="try.shp"
    file='9_Perimeters.shp'
    data=gpd.read_file(file)
    for index, row in data.iterrows():
        poly = row['geometry']
    #FQFQFQ: See Coordinate  -2096325.000000 -2092695.000000 2037105.000000 2040735.000000 
    BunsiteBound=(-2096325.0,2037105.0,-2092695.0,2040735.0)
    print(f"FQ",poly.bounds)
    scale = get_scale(BunsiteBound, meter=1)
    
    area_map,mn,mx,sh = get_raster(data, scale, BunsiteBound)
    img=area_map
    matrix = np.full(img.shape, 0, dtype=np.uint8)
    matrix[img == -1] = 255
    # area_maps = get_all_area_maps("./test_maps/")
    # area_map = area_maps[0]
    # start_point = (12, 28)
    # matrix = bous_preprocess(area_map)
    #print(matrix)
    print("times ↓")
    coverage_path,critical,cri_end = do_everything(matrix, start_point, break_after_bous=False)
    end_point = coverage_path[-1]
    print("\nmetrics ↓")
    imshow(matrix, figsize=(8, 8), cmap="cividis")   
    
    #printer(coverage_metrics(area_map, coverage_path))
                  #Shift + Tab
    imshow_scatter(critical, alpha=0.4, color="red",s=20)
    imshow_scatter(cri_end, alpha=0.4, color="black",s=50)
    
    
    x,y = np.array(coverage_path).T
    print(len(x),x,y)
    count=0
    for i in range(int(len(x)/400)):
         imshow(matrix, figsize=(8, 8), cmap="cividis")   
         imshow_scatter([start_point],color="lightgreen")
         imshow_scatter([end_point],color="red")
         imshow_scatter(critical, alpha=0.4, color="red",s=20)
         imshow_scatter(cri_end, alpha=0.4, color="black",s=50)
         #imshow_scatter(coverage_path, alpha=0.4, color="black",s=5)
         plt.plot(y[0:i*400+2],x[0:i*400+2])
         plt.show()
    imshow(matrix, figsize=(8, 8), cmap="cividis")   
    imshow_scatter(critical, alpha=0.4, color="red",s=20)
    imshow_scatter(cri_end, alpha=0.4, color="black",s=50)
    plt.plot(y,x)
    plt.show()
    #count=count+1
    #print(count)
    #
# plt.plot(y,x)
# plt.show()

# a=np.array([[[0,0,255],[0,0,1],[0,0,255]],[[0,0,0],[0,0,0],[0,0,255]],[[0,0,255],[0,0,0],[0,0,255]]])
# print(a.shape)
# a=np.array(a).mean(axis=2)==85
# b=np.int8(np.zeros(a.shape))
# b[a]=1
# c=np.array([[1,2],[3,4],[2,5],[1,4]])
# print(c.shape[1])
# h=5
# w=5
# x,y=c.T
# print(x,y)
# print(c)
# is_within_bounds = (x >= 0) & (x < h) & (y >= 0) & (y < w)
# print(is_within_bounds)
# x=np.clip(x.copy(),0,h-1)
# y=np.clip(y.copy(),0,w-1)
# bt_cond_points = {
#         "r":  lambda p: p + np.array([[0,1]]), # right
#         "tr": lambda p: p + np.array([[-1,1]]), # top-right
#         "t":  lambda p: p + np.array([[-1,0]]), # top
#         "tl": lambda p: p + np.array([[-1,-1]]), # top-left
#         "l":  lambda p: p + np.array([[0,-1]]), # left
#         "bl": lambda p: p + np.array([[1,-1]]), # bottom-left
#         "b":  lambda p: p + np.array([[1,0]]), # bottom
#         "br": lambda p: p + np.array([[1,1]]), # bottom-right
#     }
# print({k:bt_cond_points[k](c) for k in bt_cond_points })
