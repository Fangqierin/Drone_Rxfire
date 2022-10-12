import numpy as np
#from common_helpers import is_valid
import matplotlib.pyplot as plt
import matplotlib
import matplotlib.colors as mcolors
import matplotlib.patches as mpatches




def imshow_scatter(path, color="orange", alpha=1, s=20):
    """
    Prints the points in the path
    """
    x,y = np.array(path).T
    print(f"path",path)
    #print(f"x,y",x,y)
    plt.scatter(x,y, color=color, alpha=alpha, s=s)

def imshow(area_map,r=1,c=1,i=1,figsize=(5,5),cmap="viridis"):
    """
    Display with no interpolation.
    """
    # if r < 2 and c < 2 or i == 1:
    #     plt.figure(figsize=figsize)
    #plt.subplot(r,c,i)
    my_cmap = matplotlib.colors.ListedColormap(['r', 'g', 'b','r','g'])
    my_cmap.set_bad(color='w', alpha=0)
    lx,ly=area_map.shape
    fig, ax = plt.subplots(1, 1, tight_layout=True)
    for x in range(lx):
        ax.axvline(x, lw=0.5, color='k')#, zorder=7)
    for y in range(ly):
        ax.axhline(y, lw=0.5, color='k')#, zorder=7)
    #ax.imshow(area_map.T,interpolation='none',cmap=my_cmap,origin='lower')
    neg=ax.imshow(area_map.T,interpolation='none',cmap=cmap,origin='lower',extent=[0, lx, 0, ly],zorder=0)
    # plt.xlim(0, lx-1) 
    # plt.ylim(0,ly-1)
    #plt.axis('off');
    fig.colorbar(neg)

    return ax

plt.rcParams["font.family"] = "Times New Roman"
plt.rcParams.update({'font.size': 10})

def imshow_EFA(area_map,r=1,c=1,i=1,figsize=(5,5),cmap="viridis"):
    """
    Display with no interpolation.
    """
    # if r < 2 and c < 2 or i == 1:
    #     plt.figure(figsize=figsize)
    #plt.subplot(r,c,i)
    
    plt.rcParams['font.family'] = 'serif'
    plt.rcParams['font.serif'] = ['Times New Roman'] + plt.rcParams['font.serif']
    plt.rcParams.update({'font.size': 10})
    my_cmap = matplotlib.colors.ListedColormap(['r', 'g', 'b','r','g'])
    my_cmap.set_bad(color='w', alpha=0)
    lx,ly=area_map.shape
    fig, ax = plt.subplots(1, 1, tight_layout=True)
    for x in range(lx):
        ax.axvline(x, lw=0.5, color='k')#, zorder=7)
    for y in range(ly):
        ax.axhline(y, lw=0.5, color='k')#, zorder=7)
    #ax.imshow(area_map.T,interpolation='none',cmap=my_cmap,origin='lower')
    norm = mcolors.DivergingNorm(vmin=0, vmax = 60, vcenter=20)

    map=ax.imshow(area_map.T,interpolation='none',cmap='hot',origin='lower',extent=[0, lx, 0, ly],zorder=0,norm=norm)
    # plt.xlim(0, lx-1) 
    #interpolation = 'bilinear
    # plt.ylim(0,ly-1)
    #plt.axis('off');
    #plt.title('Fire Arrival Time (minute)')
    cbar=fig.colorbar(map,extend='max')
    cbar.set_label(label='Fire Arrival Time (Minute)')
    
    plt.xlabel('X (10 Meters)')
    plt.ylabel('Y (10 Meters)')
    #view_colormap('cubehelix')
    fig.set_size_inches(4, 4)

    #fig.set_clim(0,60)

    return ax



def DrawTask(tasks, EFAM):
    TaskMap=np.full(EFAM.shape, -1)
    codict={'BM':0,'FT':1,'FI':2, 'B':3}
    for key, mm in tasks.items():
        x,y=key
        if len(list(mm.keys()))>1:
            TaskMap[x,y]=codict['B']
        else:
            m=list(mm.keys())[0]
            TaskMap[x,y]=codict[m]
    imshow_Task(TaskMap)
    
    #plt.show()


def imshow_Task(area_map,r=1,c=1,i=1,figsize=(5,5),cmap="viridis"):
    """
    Display with no interpolation.
    """
    # if r < 2 and c < 2 or i == 1:
    #     plt.figure(figsize=figsize)
    #plt.subplot(r,c,i)
    plt.rcParams['font.family'] = 'serif'
    plt.rcParams['font.serif'] = ['Times New Roman'] + plt.rcParams['font.serif']
    my_cmap = matplotlib.colors.ListedColormap(['r', 'g', 'b','r','g'])
    my_cmap.set_bad(color='w', alpha=0)
    lx,ly=area_map.shape
    fig, ax = plt.subplots(1, 1, tight_layout=True)
    for x in range(lx):
        ax.axvline(x, lw=0.5, color='k')#, zorder=7)
    for y in range(ly):
        ax.axhline(y, lw=0.5, color='k')#, zorder=7)
    #ax.imshow(area_map.T,interpolation='none',cmap=my_cmap,origin='lower')
    values = np.unique(area_map.ravel())

    im=ax.imshow(area_map.T,interpolation='none',cmap=cmap,origin='lower',extent=[0, lx, 0, ly],zorder=0)
    # plt.xlim(0, lx-1) 
    # plt.ylim(0,ly-1)
    #plt.axis('off');
    colors = [ im.cmap(im.norm(value)) for value in values]
    lab=['BM', 'FT', 'FI','BM&FT']
    patches = [ mpatches.Patch(color=colors[i], label=lab[i] ) for i in range(len(values)) ]
    #plt.legend(handles=patches, bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0 )
    plt.legend(handles=patches, loc=1, borderaxespad=0 )

    #plt.title('Mission of Tasks in 20 Minutes')
    #fig.colorbar(neg)
    fig.set_size_inches(4, 4)
    plt.xlabel('X (10 Meters)')
    plt.ylabel('Y (10 Meters)')
    return ax



def cluster_imshow(area_map,sc=1,r=1,c=1,i=1,figsize=(5,5),cmap="viridis"):
    """
    Display with no interpolation.
    """
    sc=10
    if r < 2 and c < 2 or i == 1:
        plt.figure(figsize=figsize)
    plt.subplot(r,c,i)
    my_cmap = matplotlib.colors.ListedColormap(['r', 'g', 'b','r','g'])
    my_cmap.set_bad(color='w', alpha=0)
    lx,ly=area_map.shape
    fig, ax = plt.subplots(1, 1, tight_layout=True)
    # for x in range(lx):
    #     ax.axvline(x, lw=0.5, color='k')#, zorder=7)
    # for y in range(ly):
    #     ax.axhline(y, lw=0.5, color='k')#, zorder=7)
    for x in range(int(np.ceil(lx/sc))):
        ax.axvline(x*sc, lw=3, color='k')#, zorder=7)
    for y in range(int(np.ceil(ly/sc))):
        ax.axhline(y*sc, lw=3, color='k')#, zorder=7)
    #ax.imshow(area_map.T,interpolation='none',cmap=my_cmap,origin='lower')
    ax.imshow(area_map.T,cmap=cmap,interpolation='none',extent=[0,lx,0,ly],origin='lower',zorder=0)
    plt.rcParams["font.family"] = "Times New Roman"
    plt.xlim(0, 40) 
    plt.ylim(20,50)
    plt.xlabel(r'$X\  ( 10 m)$', fontsize=15)
    plt.ylabel(r'$Y\ ( 10 m)$ ', fontsize=15)
    plt.xticks(fontsize=15)
    plt.yticks(fontsize=15)
    #ax.set_xticks(list(range(0,lx,10)), labelsize=12)
    # print(list(range(0,lx,5)))
    # print(list((range(0,lx*10,50))))
    #x_label_list=list(range(0,lx*10,100))
    #print(len(list(range(0,lx,10))),list(range(0,lx,10)),len(x_label_list),x_label_list)
    #ax.set_xticklabels(x_label_list)
    #ax.set_yticks(list(range(0,ly,20)), "Times New Roman" )
    #y_label_list=list(range(0,ly,5))
    #ax.set_yticklabels(y_label_list)
    #plt.axis('off');
    return ax


def bcd_preprocess(area_map):
    """
    Returns matrix that in the form
    that is read by the main algorithm.
    """
    matrix = np.full(area_map.shape, 0, dtype=np.uint8)
    matrix[area_map == 0] = 255
    return matrix

def is_valid_vectorized(coords, matrix,obstacle):
    assert coords.shape[1] == 2
    h,w = matrix.shape
    x,y = coords.T
    is_within_bounds = (x >= 0) & (x < h) & (y >= 0) & (y < w)
    x = np.clip(x.copy(), 0, h-1)
    y = np.clip(y.copy(), 0, w-1)
    #is_not_on_obstacle=True
    #is_in_mission=(matrix[x,y]!=obstacle[0])
    is_not_on_obstacle=(matrix[x,y]!=obstacle[0])
    for obs in obstacle:
        is_not_on_obstacle=is_not_on_obstacle & (matrix[x,y] != obs)
    #is_not_on_obstacle = (matrix[x,y] != 0) &  (matrix[x,y] != 150)
    return is_within_bounds & is_not_on_obstacle

def backtracking_list(memory, diameter, matrix, x, y,obstacle):
    
    bt_cond_points = {
        "r":  lambda p: p + np.array([[0,diameter]]), # right
        "tr": lambda p: p + np.array([[-1*diameter,diameter]]), # top-right
        "t":  lambda p: p + np.array([[-1*diameter,0]]), # top
        "tl": lambda p: p + np.array([[-1*diameter,-1*diameter]]), # top-left
        "l":  lambda p: p + np.array([[0,-1*diameter]]), # left
        "bl": lambda p: p + np.array([[diameter,-1*diameter]]), # bottom-left
        "b":  lambda p: p + np.array([[diameter,0]]), # bottom
        "br": lambda p: p + np.array([[diameter,diameter]]), # bottom-right
    }
    memory_ = np.array(memory)
    assert memory_.shape[1] == 2, "you've messed up something"
    
    eight_di = {k: bt_cond_points[k](memory_) for k in bt_cond_points}
    is_valid_eight = {k: is_valid_vectorized(eight_di[k], matrix, obstacle) for k in eight_di}
    cond_a = np.int0(is_valid_eight["r"] & ~is_valid_eight["br"])
    cond_b = np.int0(is_valid_eight["r"] & ~is_valid_eight["tr"])
    cond_c = np.int0(is_valid_eight["l"] & ~is_valid_eight["bl"])
    cond_d = np.int0(is_valid_eight["l"] & ~is_valid_eight["tl"])
    cond_e = np.int0(is_valid_eight["b"] & ~is_valid_eight["bl"])
    cond_f = np.int0(is_valid_eight["b"] & ~is_valid_eight["br"])
    μ_of_s = (cond_a + cond_b+ cond_c + cond_d + cond_e + cond_f)
     
    backtrack_points =  memory_[μ_of_s > 0]
    if backtrack_points.shape[0] == 0:
        backtrack_points = memory_[is_valid_eight["r"] | is_valid_eight["l"] |
                is_valid_eight["t"] | is_valid_eight["b"]]
        
    if backtrack_points.shape[0] == 0:
        return [], True
    else:
        closest_point_idx = ((backtrack_points - np.array([x,y]))**2).sum(axis = 1).argmin()
        return tuple(backtrack_points[closest_point_idx]), False


# def visit(matrix, x, y, memory):
#     matrix[(x,y)] = 150 # 150 == visited
#     memory.append((x, y))
#     return x,y

# def move_like_oxen(matrix, diameter, x, y, memory):
#     # TODO :: Variable diameter support
#     udlr = {
#         "u": lambda x,y : (x-diameter,y),
#         "d": lambda x,y : (x+diameter,y),
#         "l": lambda x,y : (x,y-diameter),
#         "r": lambda x,y : (x,y+diameter)
#     }
#     u = "u";d = "d";r = "r";l = "l"
#     visit(matrix, x, y, memory)
#
#     while True:
#         dir_ = [u,d,r,l]
#         while len(dir_) > 0:
#             d_ = dir_.pop(0)
#             x_, y_ = udlr[d_](x,y)
#             if is_valid((x_,y_), matrix, [0, 150]):
#                 x, y = visit(matrix, x_, y_, memory)
#                 break
#             elif d_ == l:
#                 return x, y