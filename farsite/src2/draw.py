import numpy as np
import matplotlib.pyplot as plt 
import pandas as pd
import sys
import re
#import seaborn as sns
path='../examples/Panther/test1177973/output/test1177973_NTFBIgnition_'
#path='/home/fangqiliu/Downloads/farsite/examples/Panther/test1177973/output/test1177973_NTFBIgnition_'
# arr_time=pd.read_csv(path+'ArrivalTime'+'.asc')
# print(arr_time)

check_list=['ArrivalTime','CrownFire','FlameLength','HeatPerUnitArea','Intensity','SpotGrid','SpreadRate']
#check_list=['SpotGrid']
count = 0
# Strips the newline character
for name in check_list:
    data=[]
    file=open(f"{path}{name}.asc")
    Lines = file.readlines()
    for line in Lines:
        
        if re.match("[A-Z|a-z]",str(line))!=None:
            k=re.split(' |\n',line)
            #print(k,k[0],'\n',len(k))
        else:
            k=[float(x) for x in line.split(' ')[:-1]]
            k=[np.max([x,-1]) for x in k ]
            data.append(k)
            #print(k,k[0],'\n',len(k))
            count += 1
    #print(np.max([-3,-2]))
    plt.imshow(data, cmap='hot', interpolation='nearest')
    plt.colorbar()
    plt.ylabel(name)
    plt.show()
    plt.savefig(f"../../Image/{name}4.eps")
    plt.clf()






