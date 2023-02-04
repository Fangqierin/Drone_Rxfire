"""
Some helper functions that are often used.
"""
import numpy as np
import matplotlib.pyplot as plt
from collections import defaultdict
import math
#from .constants import OB, NO
import re
Winds=[5,10,15]
STt=[1,20,40,60]
Unit=3
for Unit in [1,2]:
    STtime=60
    StReward=defaultdict(dict)
    StMiss=defaultdict(dict)
    StRuntime=defaultdict(dict)
    for wind in Winds:
        for ST in STt:
            STtime=ST
            if Unit==3:
                if  STtime==1 or wind in [20,25]:
                    Simfile=f"./Results/Simple222_{Unit}_{wind}_{STtime}"
            else:
                Simfile=f"./Results/Simple14_{Unit}_{wind}_{STtime}"
            #print(f"Simfile {Simfile}")
            log=open(Simfile,"r")
            filedata = log.readlines()
            SumRreward=defaultdict(dict)
            SumMiss=defaultdict(dict)
            Runtime=defaultdict(dict)
            for line in filedata:
                if re.match(r'Sum', line):
                    itms=line[:-1].split(' ')
                    #print(itms)
                    TKv=int(itms[1])
                    WPv=int(itms[2])
                    FPv=int(itms[3])
                    seev=int(itms[4])
                    SumRreward[(TKv,WPv,FPv,seev)]=float(itms[5]) # sumreward
                    SumMiss[(TKv,WPv,FPv,seev)]=int(float(itms[6])/10)
                    Runtime[(TKv,WPv,FPv,seev)]=float(itms[-1])
            StReward[(wind,STtime)]=SumRreward
            StMiss[(wind, STtime)]=SumMiss
            StRuntime[(wind, STtime)]=Runtime
            log.close()  
    #print(SumRreward)
    #print(StRuntime)
    # for k, i in StRuntime.items():
    #     for m, a in i.items():
    #         print(k,m,a)
    plt.rcParams['font.family'] = 'serif'
    plt.rcParams['font.serif'] = ['Times New Roman'] + plt.rcParams['font.serif']
    plt.rcParams.update({'font.size': 13})
    
    Label=['WPC-DFP', 'UTA-DFP', 'ATA-DFP', 'VD-DFP']
    
    #conf_co=1.645
    conf_co=1.960
    wind=15
    Label=[ 'UTA-DL_Dis', 'UTA-NN', 'UTA-RB','UTA-DL_NN','UTA-DL_RB','UTA-DFP','UTA-DFP']
    Label=[ '5 MPH wind', '10 MPH wind', '15 MPH wind']
    barWidth = 0.2
    Brs=[np.arange(len(STt))]
    
    for i in range(4):
        Brs.append([x + barWidth for x in Brs[-1]])
    i=0
    colors=['plum','orange','lightblue']
    for wind in Winds:
        for fpm in [5]:
        #for tav in TAKM:
            print(f"{wind} ")
            Rd=[np.mean([StRuntime[wind,st][2,2,fpm,k] for k in range(10)])  for st in STt]
            Rdst=np.array([np.std([StRuntime[wind,st][2,2,fpm,k] for k in range(10)]) for st in STt])
            Rder=conf_co*(Rdst/math.sqrt(10))
            #Rd=[StMiss[wind,st][2,1,fpm] for st in STt]
            #print(f" {wind} {st} {tav} {} {StMiss[wind,st][tav,1,0]}")
            #plt.bar(Brs[i], Rd, width = barWidth, edgecolor ='grey', label =Label[fpm])
            plt.bar(Brs[i], Rd, width = barWidth, yerr=Rder, align='center',capsize=5, edgecolor ='grey', label =Label[i],color=colors[i])#,color=plt.rcParams['axes.color_cycle'][2])
            i=i+1
    plt.legend()
    plt.rcParams["image.cmap"] = "Accent"
    plt.rcParams['axes.prop_cycle'] = plt.cycler(color=plt.cm.Set1.colors)
    
    plt.xlabel('Plan Duration (Minute)')
    plt.ylabel('Running Time of UTA-DFP (s)')
    #plt.rcParams['axes.color_cycle'] 
    #plt.yscale('symlog')
    plt.xticks([r + barWidth for r in range(len(STt))],['0-20', '20-40', '40-60', '60-80'])
    plt.grid( linestyle = '--', linewidth = 1)
    plt.savefig(f"./Results/FP_Runtime_{Unit}.eps", bbox_inches='tight')
    plt.show()
    plt.clf() 







