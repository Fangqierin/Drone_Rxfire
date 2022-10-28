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
STt=[20,40,60]

Nums=[8,10,12,14,16]
#Nums=[14]
Unit=1
STtime=60
StReward=defaultdict(dict)
StMiss=defaultdict(dict)

for num in Nums:
    for ST in [60]:
        #STtime=Nums
        Simfile=f"./Results/SimpleFQ_{Unit}_{num}_{STtime}"
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
                #print(TKv,WPv,FPv,seev)
                SumMiss[(TKv,WPv,FPv,seev)]=int(float(itms[6])/10)
                Runtime[(TKv,WPv,FPv,seev)]=float(itms[-1])
        StReward[(num,STtime)]=SumRreward
        StMiss[(num, STtime)]=SumMiss
        log.close()  
#print(SumRreward)
plt.rcParams['font.family'] = 'serif'
plt.rcParams['font.serif'] = ['Times New Roman'] + plt.rcParams['font.serif']
plt.rcParams.update({'font.size': 13})

barWidth = 0.25
fig = plt.subplots(figsize =(6, 6))

TAKM=[2,3,4]
# Set position of bar on X axis
Brs=[np.arange(len(Nums))]
#br1 = np.arange(len(STt))
for i in TAKM:
    Brs.append([x + barWidth for x in Brs[-1]])
Label=['WPC-DFP', 'UTA-DFP', 'ATA-DFP', 'VD-DFP']
Colors=['b','g','y']

conf_co=1.645
#conf_co=1.960
wind=10
i=0
st=60
for tav in TAKM:
    #print([StReward[num,st] for num in Nums] )
    for k in range(10):
        print(tav, k, [StReward[num,st][tav,1,5,k] for num in Nums] )

    Rd=[np.mean([StReward[num,st][tav,1,5,k] for k in range(10)]) for num in Nums]
    Rdst=np.array([np.std([StReward[num,st][tav,1,5,k] for k in range(10)]) for num in Nums])
    Rder=conf_co*(Rdst/math.sqrt(10))
    #print(f"see Rd {Rd}")
    #print(f" {wind} {st} {tav} {} {StMiss[wind,st][tav,1,0]}")
    plt.bar(Brs[i], Rd, width = barWidth, yerr=Rder, align='center',capsize=5, edgecolor ='grey', label =Label[tav-1])
    i=i+1
plt.legend()
#Adding Xticks
plt.xlabel('Number of Drones')
plt.ylabel('Total Reward')
#plt.yscale('symlog')
#plt.xticks([r + barWidth for r in range(len(Nums))],['4', '6', '8', '10'])
plt.grid( linestyle = '--', linewidth = 1)
plt.xticks([r + barWidth for r in range(len(Nums))],Nums)

plt.savefig(f"./Results/TA_Num_{Unit}_{wind}.eps", bbox_inches='tight')
#plt.grid( linestyle = '--', linewidth = 1)
#plt.show()
plt.clf()

################## Miss 
i=0
for tav in TAKM:
    #print([StReward[num,st] for num in Nums] )
    # for k in range(10):
    #     print(tav, k, [StMiss[num,st][tav,1,5,k] for num in Nums] )
    Rd=[np.mean([StMiss[num,st][tav,1,5,k] for k in range(10)]) for num in Nums]
    Rdst=np.array([np.std([StMiss[num,st][tav,1,5,k] for k in range(10)]) for num in Nums])
    Rder=conf_co*(Rdst/math.sqrt(10))
    #print(f"see Rd {Rd}")
    #print(f" {wind} {st} {tav} {} {StMiss[wind,st][tav,1,0]}")
    plt.bar(Brs[i], Rd, width = barWidth, yerr=Rder, align='center',capsize=5, edgecolor ='grey', label =Label[tav-1])
    i=i+1
plt.legend()
#Adding Xticks
plt.xlabel('Number of Drones')
plt.ylabel('Total Missing Subtasks')
#plt.yscale('symlog')
plt.xticks([r + barWidth for r in range(len(Nums))],Nums)
plt.grid( linestyle = '--', linewidth = 1)

plt.savefig(f"./Results/TA_Num_miss_{Unit}_{wind}.eps", bbox_inches='tight')
#plt.show()
plt.clf()
#####################################################
Label0=['WPC-DFP', 0, 'UTA-', 'ATA-', 'VD-']
Label1=[ 'UTA-DL_Dis', 'NN', 'RM','DNN','DRM','DFP','DFP']

Label=[ 'UTA-DL_Dis', 'UTA-NN', 'UTA-RM','UTA-DNN','UTA-DRM','UTA-DFP','UTA-DFP']


FPM=[5,1,2,3,4]
#FPM=[5,2,4]
FPM=[5,2]
barWidth = 0.15
Brs=[np.arange(len(Nums))]
#br1 = np.arange(len(STt))
for i in FPM:
    Brs.append([x + barWidth for x in Brs[-1]])
plt.clf()
i=0
st=60
for tav in [2,4]:
    for fpm in FPM:
        Rd=[np.mean([StReward[num,st][tav,1,fpm,k] for k in range(10)])  for num in Nums]
        print(Rd)
        #print(f" {wind} {st} {tav} {} {StMiss[wind,st][tav,1,0]}")
        Rdst=np.array([np.std([StReward[num,st][2,1,fpm,k] for k in range(10)]) for num in Nums])
        Rder=conf_co*(Rdst/math.sqrt(10))
        #plt.plot(Nums, Rd)
        #plt.errorbar(Nums, Rd, Rder, marker='s', mfc='red', mec='green', ms=20, mew=4)
        plt.errorbar(Nums, Rd, Rder, marker='s',  ms=8, mew=4,label =f"{Label0[tav]}{Label1[fpm]}")

    #plt.plot(list(range(len(Nums))), Rd)
    #plt.bar(Brs[i], Rd, width = barWidth, yerr=Rder, align='center',capsize=5, edgecolor ='grey', label =Label[fpm])
    #plt.bar(Brs[i], Rd, width = barWidth, edgecolor ='grey', label =Label[fpm])
    i=i+1
plt.legend()
plt.xlabel('Number of Drones')
plt.ylabel('Total Reward')
#plt.yscale('symlog')
#plt.xticks([r + barWidth for r in range(len(Nums))],Nums)
plt.grid( linestyle = '--', linewidth = 1)
#plt.yscale('symlog')

plt.savefig(f"./Results/FP_Num_{Unit}_{wind}.eps", bbox_inches='tight')
plt.show()

############################
plt.clf()
i=0
# for fpm in FPM:
# #for tav in TAKM:
#     Rd=[np.mean([StMiss[wind,st][2,1,fpm,k] for k in range(10)])  for st in STt]
#     Rdst=np.array([np.std([StMiss[wind,st][2,1,fpm,k] for k in range(10)]) for st in STt])
#     Rder=conf_co*(Rdst/math.sqrt(10))
#     #Rd=[StMiss[wind,st][2,1,fpm] for st in STt]
#     #print(f" {wind} {st} {tav} {} {StMiss[wind,st][tav,1,0]}")
#     #plt.bar(Brs[i], Rd, width = barWidth, edgecolor ='grey', label =Label[fpm])
#     plt.bar(Brs[i], Rd, width = barWidth, yerr=Rder, align='center',capsize=5, edgecolor ='grey', label =Label[fpm])
#     i=i+1
# plt.legend()
# plt.xlabel('Number of Drones')
for fpm in FPM:
#for tav in TAKM:
    Rd=[np.mean([StMiss[num,st][2,1,fpm,k] for k in range(10)])  for num in Nums]
    #print(f" {wind} {st} {tav} {} {StMiss[wind,st][tav,1,0]}")
    Rdst=np.array([np.std([StMiss[num,st][2,1,fpm,k] for k in range(10)]) for num in Nums])
    Rder=conf_co*(Rdst/math.sqrt(10))
    #plt.bar(Brs[i], Rd, width = barWidth, yerr=Rder, align='center',capsize=5, edgecolor ='grey', label =Label[fpm])
    plt.errorbar(Nums, Rd, Rder, marker='s',  ms=8, mew=4,label =Label[fpm])

    #plt.bar(Brs[i], Rd, width = barWidth, edgecolor ='grey', label =Label[fpm])
    i=i+1
plt.legend()
plt.xlabel('Number of Drones')
plt.ylabel('Total Missing Subtasks')
#plt.xlim(0)
#plt.yscale('symlog')
#plt.xticks([r + barWidth for r in range(len(Nums))],Nums)
plt.grid( linestyle = '--', linewidth = 1)
#plt.yscale('symlog')
plt.savefig(f"./Results/FP_Num_miss_{Unit}_{wind}.eps", bbox_inches='tight')
plt.show()







