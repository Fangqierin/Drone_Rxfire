"""
Some helper functions that are often used.
"""
import numpy as np
import matplotlib.pyplot as plt
from collections import defaultdict
import math
#from .constants import OB, NO
import re
Winds=[10]
STt=[1,20,40,60]
Unit=2
STtime=60
StReward=defaultdict(dict)
StMiss=defaultdict(dict)

for wind in [10]:#Winds:
    for ST in STt:
        STtime=ST
        Simfile=f"./Results/Simple14_{Unit}_{wind}_{STtime}"
        log=open(Simfile,"r")
        filedata = log.readlines()
        
        SumMiss=defaultdict(dict)
        Runtime=defaultdict(dict)
        SumRreward=defaultdict(dict)
        for line in filedata:
            if re.match(r'Reward', line):
                content=line[:-1].split('; ')
                itms=content[0].split(' ')
                TKv=int(itms[1])
                WPv=int(itms[2])
                FPv=int(itms[3])
                seev=int(itms[4])
                donm=int(itms[5])
                #print(donm)
                #print(f"{(TKv,WPv,FPv,seev,donm)} {content}")
                TmpRecord={}
                for cc in content[1:-1]:
                    t, rwd,swd,p=cc.split(' ')
                    TmpRecord[int(t)]=(float(rwd),float(swd),float(p))
                SumRreward[(TKv,WPv,FPv,seev,donm)]=TmpRecord
                #print(TmpRecord)
                #print(TKv,WPv,FPv,seev)
                # SumMiss[(TKv,WPv,FPv,seev)]=int(float(itms[6])/10)
                # Runtime[(TKv,WPv,FPv,seev)]=float(itms[-1])
        StReward[STtime]=SumRreward
        # StMiss[(wind, STtime)]=SumMiss
        log.close() 

plt.rcParams['font.family'] = 'serif'
plt.rcParams['font.serif'] = ['Times New Roman'] + plt.rcParams['font.serif']
plt.rcParams.update({'font.size': 13})

################### Single drone draw reward assumulative value 
Times=list(range(1250))
Reward=[0]*len(Times)
TimRwd=dict(zip(Times,Reward))
#print(TimRwd)
STtime=60
Dones=[0,1,2,3,4,5]
donum=0
CCRreward=StReward[STtime]
conf_co=1.960
#wind=15
i=0
# for tav in TAKM:
#     print([StReward[wind,st] for st in STt] )
#     for k in range(10):
#         print(tav, k, [StReward[wind,st][tav,1,5,k] for st in STt] )
#     Rd=[np.mean([StReward[wind,st][tav,1,5,k] for k in range(10)]) for st in STt]
#     Rdst=np.array([np.std([StReward[wind,st][tav,1,5,k] for k in range(10)]) for st in STt])
#     Rder=conf_co*(Rdst/math.sqrt(10))
#     #print(f"see Rd {Rd}")
#     #print(f" {wind} {st} {tav} {} {StMiss[wind,st][tav,1,0]}")
#     plt.bar(Brs[i], Rd, width = barWidth, yerr=Rder, align='center',capsize=5, edgecolor ='grey', label =Label[tav-1])
#     i=i+1
# plt.legend()
seed=8
TAKM=[2,3,4]
Colors=['blue','orange', 'green']
M=['o','*','1','<','+']
Label=['WPC-DFP', 'UTA-DFP', 'ATA-DFP', 'VD-DFP']

c=0
tt=[(t+(STtime)*60)/60 for t in Times]
for tav in TAKM:
    Acrwd=0
    TimRwd=dict(zip(Times,Reward))
    #allt=sorted(list(Allv.keys()))
    for i in Times:
        for donm in Dones:
            #print(donm,CCRreward[(tav,1,5,seed,donm)].keys())
            if CCRreward[(tav,2,5,seed,donm)].get(i)!=None:
                #print(donm,tav,CCRreward[(tav,1,5,seed,donm)][i][0])
                Acrwd=Acrwd+CCRreward[(tav,2,5,seed,donm)][i][0]
                #print(f"why {tav} {seed} {donm}")
            
        # if i in allt:
        #     Acrwd=Acrwd+Allv[i][0]
        TimRwd[i]=Acrwd
    #print(tav,TimRwd)
    plt.plot(tt,list(TimRwd.values()),label=Label[tav-1],marker=M[c],markevery=50)
    c=c+1
plt.legend()
plt.xlabel('Time (Minute)')
plt.ylabel('Accumulative Reward')
plt.grid( linestyle = '--', linewidth = 1)
plt.xlim(STtime,STtime+21)
plt.xticks(np.arange(min(tt), max(tt)+1, 2))
plt.savefig(f"./Results/TA_comp_Snt_{wind}_{STtime}.eps", bbox_inches='tight')
#plt.show()
plt.clf()

c=0
for tav in TAKM:
    Acrwd=0
    TimRwd=dict(zip(Times,Reward))
    #allt=sorted(list(Allv.keys()))
    for i in Times:
        for donm in Dones:
            #print(donm,CCRreward[(tav,1,5,seed,donm)].keys())
            if CCRreward[(tav,2,5,seed,donm)].get(i)!=None:
                #print(donm,tav,CCRreward[(tav,1,5,seed,donm)][i][0])
                Acrwd=Acrwd+CCRreward[(tav,1,5,seed,donm)][i][2]/10
                #print(f"why {tav} {seed} {donm}")
        # if i in allt:
        #     Acrwd=Acrwd+Allv[i][0]
        TimRwd[i]=Acrwd
    #print(tav,TimRwd)
    plt.plot(tt,list(TimRwd.values()),label=Label[tav-1],marker=M[c],markevery=50)
    c=c+1
plt.legend()
plt.xlabel('Time (Minute)')
plt.ylabel('Accumulative Missing Subtasks')
plt.grid( linestyle = '--', linewidth = 1)
plt.xlim(STtime,STtime+21)
plt.xticks(np.arange(min(tt), max(tt)+1, 2))
plt.savefig(f"./Results/TA_miss_Snt_{wind}_{STtime}.eps", bbox_inches='tight')

#plt.show()
plt.clf()
#######################################################################
Label=[ 'UTA-DL_Dis', 'UTA-NN', 'UTA-RB','UTA-DL_NN','UTA-DL_RB','UTA-DFP','UTA-DFP']
FPM=[5,1,2,3,4]
c=0
for fpm in FPM:
    Acrwd=0
    TimRwd=dict(zip(Times,Reward))
    #allt=sorted(list(Allv.keys()))
    for i in Times:
        for donm in Dones:
            #print(donm,CCRreward[(tav,1,5,seed,donm)].keys())
            if CCRreward[(2,2,fpm,seed,donm)].get(i)!=None:
                #print(donm,tav,CCRreward[(tav,1,5,seed,donm)][i][0])
                Acrwd=Acrwd+CCRreward[(2,1,fpm,seed,donm)][i][0]
                #print(f"why {tav} {seed} {donm}")
        # if i in allt:
        #     Acrwd=Acrwd+Allv[i][0]
        TimRwd[i]=Acrwd
    #print(tav,TimRwd)
    plt.plot(tt,list(TimRwd.values()),label=Label[fpm],marker=M[c],markevery=50)
    c=c+1
plt.legend()
plt.xlabel('Time (Minute)')
plt.ylabel('Accumulative Missing Subtasks')
#plt.yscale('symlog')
plt.grid( linestyle = '--', linewidth = 1)
plt.xlim(STtime,STtime+21)
plt.xticks(np.arange(min(tt), max(tt)+1, 2))
plt.savefig(f"./Results/FP_comp_Snt_{wind}_{STtime}.eps", bbox_inches='tight')
plt.show()
plt.clf()



c=0
for fpm in FPM:
    Acrwd=0
    TimRwd=dict(zip(Times,Reward))
    #allt=sorted(list(Allv.keys()))
    for i in Times:
        for donm in Dones:
            #print(donm,CCRreward[(tav,1,5,seed,donm)].keys())
            if CCRreward[(2,2,fpm,seed,donm)].get(i)!=None:
                #print(donm,tav,CCRreward[(tav,1,5,seed,donm)][i][0])
                Acrwd=Acrwd+CCRreward[(2,1,fpm,seed,donm)][i][2]/10
                #print(f"why {tav} {seed} {donm}")
        # if i in allt:
        #     Acrwd=Acrwd+Allv[i][0]
        TimRwd[i]=Acrwd
    #print(tav,TimRwd)
    plt.plot(tt,list(TimRwd.values()),label=Label[fpm],marker=M[c],markevery=50)
    c=c+1
plt.legend()
plt.xlabel('Time (Minute)')
plt.ylabel('Accumulative Missing Subtasks')
plt.yscale('symlog')
plt.grid( linestyle = '--', linewidth = 1)
plt.xlim(STtime,STtime+21)
plt.ylim(0)
plt.xticks(np.arange(min(tt), max(tt)+1, 2))
plt.savefig(f"./Results/FP_miss_Snt_{wind}_{STtime}.eps", bbox_inches='tight')
plt.show()


# c=0
# for tav in TAKM:
#     Acrwd=0
#     TimRwd=dict(zip(Times,Reward))
#     #allt=sorted(list(Allv.keys()))
#     for i in Times:
#         tmpm=0
#         for donm in Dones:
#             #print(donm,CCRreward[(tav,1,5,seed,donm)].keys())
#             if CCRreward[(tav,1,5,seed,donm)].get(i)!=None:
#                 #print(donm,tav,CCRreward[(tav,1,5,seed,donm)][i][0])
#                 tmpm=tmpm+CCRreward[(tav,1,5,seed,donm)][i][2]/10
#                 #print(f"why {tav} {seed} {donm}")
#
#         # if i in allt:
#         #     Acrwd=Acrwd+Allv[i][0]
#         TimRwd[i]=tmpm
#     #print(tav,TimRwd)
#     plt.plot(tt,list(TimRwd.values()),color=Colors[c],label=Label[tav-1])
#     c=c+1
# plt.legend()
# plt.xlabel('Time (Minute)')
# plt.ylabel('Accumulative Missing Subtasks')
# plt.grid( linestyle = '--', linewidth = 1)
# plt.xlim(STtime,STtime+21)
# plt.xticks(np.arange(min(tt), max(tt)+1, 2))
# plt.show()










'''
#print(SumRreward)
plt.rcParams['font.family'] = 'serif'
plt.rcParams['font.serif'] = ['Times New Roman'] + plt.rcParams['font.serif']
plt.rcParams.update({'font.size': 10})

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

#conf_co=1.645
conf_co=1.960
wind=15
i=0
for tav in TAKM:
    print([StReward[wind,st] for st in STt] )
    for k in range(10):
        print(tav, k, [StReward[wind,st][tav,1,5,k] for st in STt] )
    Rd=[np.mean([StReward[wind,st][tav,1,5,k] for k in range(10)]) for st in STt]
    Rdst=np.array([np.std([StReward[wind,st][tav,1,5,k] for k in range(10)]) for st in STt])
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
plt.show()
plt.clf()
################## Miss 
i=0
for tav in TAKM:
    Rd=[np.mean([StMiss[wind,st][tav,1,5,k] for k in range(10)]) for st in STt]
    #print(f" {wind} {st} {tav} {} {StMiss[wind,st][tav,1,0]}")
    Rdst=np.array([np.std([StMiss[wind,st][tav,1,5,k] for k in range(10)]) for st in STt])
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
plt.show()
#####################################################


Label=[ 'UTA-DL_Dis', 'UTA-NN', 'UTA-RB','UTA-DL_NN','UTA-DL_RB','UTA-DFP','UTA-DFP']

FPM=[5,1,2,3,4]
barWidth = 0.15
Brs=[np.arange(len(STt))]
#br1 = np.arange(len(STt))
for i in FPM:
    Brs.append([x + barWidth for x in Brs[-1]])
plt.clf()
i=0
for fpm in FPM:
#for tav in TAKM:
    Rd=[np.mean([StReward[wind,st][2,1,fpm,k] for k in range(10)])  for st in STt]
    #print(f" {wind} {st} {tav} {} {StMiss[wind,st][tav,1,0]}")
    Rdst=np.array([np.std([StReward[wind,st][2,1,fpm,k] for k in range(10)]) for st in STt])
    Rder=conf_co*(Rdst/math.sqrt(10))
    plt.bar(Brs[i], Rd, width = barWidth, yerr=Rder, align='center',capsize=5, edgecolor ='grey', label =Label[fpm])
    #plt.bar(Brs[i], Rd, width = barWidth, edgecolor ='grey', label =Label[fpm])
    i=i+1
plt.legend()
plt.xlabel('Plan Duration (Minute)')
plt.ylabel('Total Reward')
#plt.yscale('symlog')
#plt.xticks([r + barWidth for r in range(len(STt))], ['0-20', '20-40', '40-60', '60-80'])
plt.grid( linestyle = '--', linewidth = 1)
plt.show()
plt.savefig(f"./Results/FP_comp_{Unit}_{wind}.eps", bbox_inches='tight')
############################
plt.clf()
i=0
for fpm in FPM:
#for tav in TAKM:
    Rd=[np.mean([StMiss[wind,st][2,1,fpm,k] for k in range(10)])  for st in STt]
    Rdst=np.array([np.std([StMiss[wind,st][2,1,fpm,k] for k in range(10)]) for st in STt])
    Rder=conf_co*(Rdst/math.sqrt(10))
    #Rd=[StMiss[wind,st][2,1,fpm] for st in STt]
    #print(f" {wind} {st} {tav} {} {StMiss[wind,st][tav,1,0]}")
    #plt.bar(Brs[i], Rd, width = barWidth, edgecolor ='grey', label =Label[fpm])
    plt.bar(Brs[i], Rd, width = barWidth, yerr=Rder, align='center',capsize=5, edgecolor ='grey', label =Label[fpm])
    i=i+1
plt.legend()
plt.xlabel('Plan Duration (Minute)')
plt.ylabel('Total Missing Subtasks')
#plt.yscale('symlog')
#plt.xticks([r + barWidth for r in range(len(STt))],['0-20', '20-40', '40-60', '60-80'])
plt.grid( linestyle = '--', linewidth = 1)
plt.show()
plt.savefig(f"./Results/FP_miss_{Unit}_{wind}.eps", bbox_inches='tight')
'''









