"""
Some helper functions that are often used.
"""
import numpy as np
import matplotlib.pyplot as plt
from collections import defaultdict
import math
#from .constants import OB, NO
import re
Winds=[5,10,15,20,25]
STt=[40]#[1,20,40,60]
Unit=1
#STtime=40
st=40
StReward=defaultdict(dict)
StMiss=defaultdict(dict)
Seed=list(range(10))
for wind in Winds:
    for ST in STt:
        STtime=ST
        # if  wind in [20,25]:
        #     Simfile=f"./Results/Simple222_{Unit}_{wind}_{STtime}"
        #     print(Simfile)
        # else:
        Simfile=f"./Results/REP_Simple23_{Unit}_{wind}_{STtime}"
        #print(Simfile)
        # # if  STtime==1 or wind in [20,25]:
        # #     Simfile=f"./Results/Simple222_{Unit}_{wind}_{STtime}"
        # #     print(Simfile)
        # # else:
        # Simfile=f"./Results/Simple14_{Unit}_{wind}_{STtime}"
        # if wind in [20,25]:
        #     Simfile=f"./Results/Simple222_{Unit}_{wind}_{STtime}"
        # else:
        #     Simfile=f"./Results/Simple14_{Unit}_{wind}_{STtime}"
        log=open(Simfile,"r")
        filedata = log.readlines()
        SumRreward=defaultdict(dict)
        SumMiss=defaultdict(dict)
        Runtime=defaultdict(dict)
        for line in filedata:
            if re.match(r'Sum', line):
                
                    #print(f"why",line)
                itms=line[:-1].split(' ')
                #print(itms)
                TKv=int(itms[1])
                WPv=int(itms[2])
                FPv=int(itms[3])
                seev=int(itms[4])
                if STtime==1 and wind==5:
                    print(f"whyyy {(TKv,WPv,FPv,seev)}")
                SumRreward[(TKv,WPv,FPv,seev)]=float(itms[5]) # sumreward
                #print(TKv,WPv,FPv,seev)
                SumMiss[(TKv,WPv,FPv,seev)]=int(float(itms[6])/10)
                Runtime[(TKv,WPv,FPv,seev)]=float(itms[-1])
        StReward[(wind,STtime)]=SumRreward
        StMiss[(wind, STtime)]=SumMiss
        log.close()  
#print(SumRreward)
plt.rcParams['font.family'] = 'serif'
plt.rcParams['font.serif'] = ['Times New Roman'] + plt.rcParams['font.serif']
plt.rcParams.update({'font.size': 13})

barWidth = 0.25
fig = plt.subplots(figsize =(6, 6))

TAKM=[2,3,4]
# Set position of bar on X axis
Brs=[np.arange(len(STt))]
#br1 = np.arange(len(STt))
for i in TAKM:
    Brs.append([x + barWidth for x in Brs[-1]])
Label=['WPC-DFP', 'UTA-DFP', 'ATA-DFP', 'VD-DFP']
Colors=['b','g','y']

conf_co=1.645
#conf_co=1.960
wind=20
'''
i=0
for tav in TAKM:
    print([StReward[wind,st] for st in STt] )
    for k in range(10):
        print(tav, k, [StReward[wind,st][tav,1,5,k] for st in STt] )
    Rd=[np.mean([StReward[wind,st][tav,1,5,k] for k in Seed]) for st in STt]
    Rdst=np.array([np.std([StReward[wind,st][tav,1,5,k] for k in Seed]) for st in STt])
    Rder=conf_co*(Rdst/math.sqrt(10))
    #print(f"see Rd {Rd}")
    #print(f" {wind} {st} {tav} {} {StMiss[wind,st][tav,1,0]}")
    plt.bar(Brs[i], Rd, width = barWidth, yerr=Rder, align='center',capsize=5, edgecolor ='grey', label =Label[tav-1])
    i=i+1
plt.legend()
#Adding Xticks
plt.xlabel('Plan Duration (Minute)')
plt.ylabel('Total Reward')
#plt.yscale('symlog')
#plt.xticks([r + barWidth for r in range(len(STt))],['0-20', '20-40', '40-60', '60-80'])
plt.grid( linestyle = '--', linewidth = 1)

plt.savefig(f"./Results/TA_comp_{Unit}_{wind}.eps", bbox_inches='tight')
#plt.grid( linestyle = '--', linewidth = 1)
#plt.show()
plt.clf()
################## Miss 
i=0
for tav in TAKM:
    Rd=[np.mean([StMiss[wind,st][tav,1,5,k] for k in Seed]) for st in STt]
    #print(f" {wind} {st} {tav} {} {StMiss[wind,st][tav,1,0]}")
    Rdst=np.array([np.std([StMiss[wind,st][tav,1,5,k] for k in Seed]) for st in STt])
    Rder=conf_co*(Rdst/math.sqrt(10))
    plt.bar(Brs[i], Rd, width = barWidth, yerr=Rder, align='center',capsize=5, edgecolor ='grey', label =Label[tav-1])
    #plt.bar(Brs[i], Rd, width = barWidth, edgecolor ='grey', label =Label[tav-1])
    i=i+1
plt.legend()
#Adding Xticks
plt.xlabel('Plan Duration (Minute)')
plt.ylabel('Total Missing Subtasks')
#plt.yscale('symlog')
#plt.xticks([r + barWidth for r in range(len(STt))],['0-20', '20-40', '40-60', '60-80'])
plt.grid( linestyle = '--', linewidth = 1)

plt.savefig(f"./Results/TA_miss_{Unit}_{wind}.eps", bbox_inches='tight')
#plt.show()
#####################################################


Label=[ 'UTA-DL_Dis', 'UTA-NN', 'UTA-RB','UTA-DL_NN','UTA-DL_RB','UTA-DFP','UTA-DFP']

FPM=[5,1,2,3,4]
barWidth = 0.15
Brs=[np.arange(len(STt))]
M=['o','*','1','<','+']
Label0=['WPC-DFP', 0, 'UTA-', 'ATA-', 'VD-']
Label1=[ 'UTA-DL_Dis', 'NN', 'RM','DNN','DRM','DFP','DFP']
i=0

#br1 = np.arange(len(STt))
for i in FPM:
    Brs.append([x + barWidth for x in Brs[-1]])
plt.clf()
i=0
#for fpm in FPM:
for tav in [2,4]:
    for fpm in [5,2]:
#for tav in TAKM:
        Rd=[np.mean([StReward[wind,st][tav,1,fpm,k] for k in Seed])  for st in STt]
        #print(f" {wind} {st} {tav} {} {StMiss[wind,st][tav,1,0]}")
        Rdst=np.array([np.std([StReward[wind,st][tav,1,fpm,k] for k in Seed]) for st in STt])
        Rder=conf_co*(Rdst/math.sqrt(10))
        #plt.bar(Brs[i], Rd, width = barWidth, yerr=Rder, align='center',capsize=5, edgecolor ='grey', label =Label[fpm])
    #plt.bar(Brs[i], Rd, width = barWidth, edgecolor ='grey', label =Label[fpm])
        if tav==2:
            line='-'
        else:
            line='--'
        plt.errorbar(STt, Rd, Rder, marker=M[i], linestyle=line, ms=5, mew=5,capsize=3,label =f"{Label0[tav]}{Label1[fpm]}")
        i=i+1
plt.legend()
plt.xlabel('Plan Duration (Minute)')
plt.ylabel('Total Reward')
#plt.yscale('symlog')
#plt.xticks([r + barWidth for r in range(len(STt))], ['0-20', '20-40', '40-60', '60-80'])
plt.grid( linestyle = '--', linewidth = 1)
plt.xticks(STt,['0-20', '20-40', '40-60', '60-80'])

plt.savefig(f"./Results/FP_comp_{Unit}_{wind}.eps", bbox_inches='tight')
plt.show()
############################
plt.clf()
i=0
#for fpm in FPM:
for tav in [2,4]:
    for fpm in [5,2]:
#for tav in TAKM:
        Rd=[np.mean([StMiss[wind,st][tav,1,fpm,k] for k in Seed])  for st in STt]
        Rdst=np.array([np.std([StMiss[wind,st][tav,1,fpm,k] for k in Seed]) for st in STt])
        Rder=conf_co*(Rdst/math.sqrt(10))
        #Rd=[StMiss[wind,st][2,1,fpm] for st in STt]
        #print(f" {wind} {st} {tav} {} {StMiss[wind,st][tav,1,0]}")
        #plt.bar(Brs[i], Rd, width = barWidth, edgecolor ='grey', label =Label[fpm])
        #plt.bar(Brs[i], Rd, width = barWidth, yerr=Rder, align='center',capsize=5, edgecolor ='grey', label =Label[fpm])
        
        if tav==2:
            line='-'
        else:
            line='--'
        plt.errorbar(STt, Rd, Rder, marker=M[i], linestyle=line, ms=5, mew=5,capsize=3,label =f"{Label0[tav]}{Label1[fpm]}")
        i=i+1
plt.legend()
plt.xlabel('Plan Duration (Minute)')
plt.ylabel('Total Missing Subtasks')
#plt.yscale('symlog')
#plt.xticks([r + barWidth for r in range(len(STt))],['0-20', '20-40', '40-60', '60-80'])
plt.xticks(STt,['0-20', '20-40', '40-60', '60-80'])

plt.grid( linestyle = '--', linewidth = 1)
plt.savefig(f"./Results/FP_miss_{Unit}_{wind}.eps", bbox_inches='tight')
plt.show()
'''
########################################################################################
FPM=[5,2]

M=['o','*','1','<','+']
Label0=['WPC-DFP', 0, 'UTA-', 'ATA-', 'VD-']
Label1=[ 'UTA-DL_Dis', 'NN', 'RM','DNN','DRM','DFP','DFP']
i=0

M=['o','*','1','<','+']

for tav in [2,4]:
    for fpm in FPM:
        Rd=[np.mean([StReward[wind,st][tav,1,fpm,k] for k in range(10)])  for wind in Winds]
        print(Rd)
        #print(f" {wind} {st} {tav} {} {StMiss[wind,st][tav,1,0]}")
        Rdst=np.array([np.std([StReward[wind,st][tav,1,fpm,k] for k in range(10)]) for wind in Winds])
        Rder=conf_co*(Rdst/math.sqrt(10))
        #plt.plot(Nums, Rd)
        #plt.errorbar(Nums, Rd, Rder, marker='s', mfc='red', mec='green', ms=20, mew=4)
        #plt.errorbar(Nums, Rd, Rder, marker='s',  ms=8, mew=4,label =f"{Label0[tav]}{Label1[fpm]}")
        if tav==2:
            line='-'
        else:
            line='--'
        plt.errorbar(Winds, Rd, Rder, marker=M[i], linestyle=line, ms=5, mew=5,capsize=3,label =f"{Label0[tav]}{Label1[fpm]}")

    #plt.plot(list(range(len(Nums))), Rd)
    #plt.bar(Brs[i], Rd, width = barWidth, yerr=Rder, align='center',capsize=5, edgecolor ='grey', label =Label[fpm])
    #plt.bar(Brs[i], Rd, width = barWidth, edgecolor ='grey', label =Label[fpm])
    i=i+1
plt.legend()
plt.xlabel('Wind Speed (mph)')
plt.ylabel('Total Reward')
#plt.yscale('symlog')
#plt.xlim(8,16)

#plt.xticks([r + barWidth for r in range(len(Nums))],Nums)
plt.grid( linestyle = '--', linewidth = 1)
plt.xticks(Winds, Winds)

#plt.yscale('symlog')
plt.savefig(f"./Results/FP_winds_{Unit}_{wind}_{st}.eps", bbox_inches='tight')
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
#for fpm in FPM:
#for tav in TAKM:
i=0
for tav in [2,4]:
    for fpm in FPM:
        Rd=[np.mean([StMiss[wind,st][tav,1,fpm,k] for k in range(10)])  for wind in Winds]
        #print(f" {wind} {st} {tav} {} {StMiss[wind,st][tav,1,0]}")
        Rdst=np.array([np.std([StMiss[wind,st][tav,1,fpm,k] for k in range(10)]) for wind in Winds])
        Rder=conf_co*(Rdst/math.sqrt(10))
        #plt.bar(Brs[i], Rd, width = barWidth, yerr=Rder, align='center',capsize=5, edgecolor ='grey', label =Label[fpm])
        #plt.errorbar(Nums, Rd, Rder, marker='s',  ms=8, mew=4,label =Label[fpm])
        if tav==2:
            line='-'
        else:
            line='--'
        plt.errorbar(Winds, Rd, Rder, marker=M[i], linestyle=line, ms=5, mew=5,capsize=3,label =f"{Label0[tav]}{Label1[fpm]}")
        
    #plt.bar(Brs[i], Rd, width = barWidth, edgecolor ='grey', label =Label[fpm])
        i=i+1
plt.legend()
plt.xlabel('Wind Speed (mph)')
plt.ylabel('Total Missing Subtasks')
#plt.xlim(7.5,16)
#plt.ylim(0)
plt.xticks(Winds, Winds)

#plt.yscale('symlog')
#plt.xticks([r + barWidth for r in range(len(Nums))],Nums)
plt.grid( linestyle = '--', linewidth = 1)
#plt.yscale('symlog')
plt.savefig(f"./Results/FP_miss_wind_{Unit}_{wind}_{st}.eps", bbox_inches='tight')
plt.show()






