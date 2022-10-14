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

STtime=60
StReward=defaultdict(dict)
StMiss=defaultdict(dict)
StRuntime=defaultdict(dict)
# for wind in Winds:
#     for ST in STt:
#         STtime=ST
#         Simfile=f"./Results/Simple12_{wind}_{STtime}"
#         log=open(Simfile,"r")
#         filedata = log.readlines()
#         SumRreward=defaultdict(dict)
#         SumMiss=defaultdict(dict)
#         Runtime=defaultdict(dict)
#         for line in filedata:
#             if re.match(r'Sum', line):
#                 itms=line[:-1].split(' ')
#                 #print(itms)
#                 TKv=int(itms[1])
#                 WPv=int(itms[2])
#                 FPv=int(itms[3])
#                 seev=int(itms[4])
#                 SumRreward[(TKv,WPv,FPv,seev)]=float(itms[5]) # sumreward
#                 SumMiss[(TKv,WPv,FPv,seev)]=int(float(itms[6])/10)
#                 Runtime[(TKv,WPv,FPv,seev)]=float(itms[-1])
#         StReward[(wind,STtime)]=SumRreward
#         StMiss[(wind, STtime)]=SumMiss
#         StRuntime[(wind, STtime)]=Runtime
#         log.close()  
# #print(SumRreward)
# print(StRuntime)
# for k, i in StRuntime.items():
#     for m, a in i.items():
#         print(k,m,a)
plt.rcParams['font.family'] = 'serif'
plt.rcParams['font.serif'] = ['Times New Roman'] + plt.rcParams['font.serif']
plt.rcParams.update({'font.size': 10})

Label=['WPC-DFP', 'UTA-DFP', 'ATA-DFP', 'VD-DFP']
Colors=['b','g','y']
#conf_co=1.645
conf_co=1.960
wind=15
Label=[ 'UTA-DL_Dis', 'UTA-NN', 'UTA-RB','UTA-DL_NN','UTA-DL_RB','UTA-DFP','UTA-DFP']
Label=[ '5 MPH wind', '10 MPH wind', '15 MPH wind']
barWidth = 0.2
Brs=[np.arange(len(STt))]
# for i in TAKM:
#     Brs.append([x + barWidth for x in Brs[-1]])
# i=0
# for wind in [5,10,15]:
#     for fpm in [5]:
#     #for tav in TAKM:
#         Rd=[np.mean([StRuntime[wind,st][2,1,fpm,k] for k in range(10)])  for st in STt]
#         Rdst=np.array([np.std([StRuntime[wind,st][2,1,fpm,k] for k in range(10)]) for st in STt])
#         Rder=conf_co*(Rdst/math.sqrt(10))
#         #Rd=[StMiss[wind,st][2,1,fpm] for st in STt]
#         #print(f" {wind} {st} {tav} {} {StMiss[wind,st][tav,1,0]}")
#         #plt.bar(Brs[i], Rd, width = barWidth, edgecolor ='grey', label =Label[fpm])
#         plt.bar(Brs[i], Rd, width = barWidth, yerr=Rder, align='center',capsize=5, edgecolor ='grey', label =Label[i])
#         i=i+1
# plt.legend()
# plt.xlabel('Plan Duration (Minute)')
# plt.ylabel('Running Time of UTA-DFP (s)')
# #plt.yscale('symlog')
# plt.xticks([r + barWidth for r in range(len(STt))],
#         ['0-20', '20-40', '40-60', '60-80'])
# plt.grid( linestyle = '--', linewidth = 1)
# #plt.savefig(f"./Results/FP_Runtime_Unit1.eps", bbox_inches='tight')
# #plt.show()
# plt.clf() 
####################################### Get the Total tasks in different seeds 
SumTasks=defaultdict(dict)
AvTasks=defaultdict(dict)
for wind in Winds:
    for ST in STt:
        STtime=ST
        Simfile=f"./Results/Simple12_{wind}_{STtime}"
        log=open(Simfile,"r")
        filedata = log.readlines()
        SumRreward=defaultdict(dict)
        SumMiss=defaultdict(dict)
        Runtime=defaultdict(dict)
        for line in filedata:
            if re.match(r'Tasks 1 1 1', line):
                allitm=line[:-3].split('; ') 
                #print(allitm)
                itms=allitm[0].split(' ')
                minus=int(allitm[-1].split(' ')[-1]) # For 1200 extra added tasks 
                #print(itms)
                TKv=int(itms[1])
                WPv=int(itms[2])
                FPv=int(itms[3])
                seev=int(itms[4])
                task=allitm[1]
                Num=int(allitm[2])-minus
                
                #if seev==0:
                    #print(itms)
                    #print(f"task {task} Num {Num}")
                if SumTasks[(wind,ST,seev)].get(task)!=None:
                    SumTasks[(wind,ST, seev)][task]=SumTasks[(wind,ST,seev)][task]+Num
                else:
                    SumTasks[(wind,ST,seev)][task]=Num
        log.close()
        for tak in SumTasks[(wind,ST,0)]:
            AvTasks[(wind,ST)][tak]=int(np.mean([SumTasks[(wind,ST,s)][tak] for s in range(10)]))
            #print((wind,ST),tak, int(np.mean([SumTasks[(wind,ST,s)][tak] for s in range(10)])))
print(SumTasks)
#print(SumRreward)
Label=[ '5 MPH wind', '10 MPH wind', '15 MPH wind']
barWidth = 0.2
Brs=[np.arange(len(STt))]
for i in Winds:
    Brs.append([x + barWidth+0.05 for x in Brs[-1]])
i=0
fig = plt.figure()
ax = fig.add_subplot(111)
for wind in [5,10,15]:
    #for tak in ['BM','FI','FT']:
    RdBM=[AvTasks[wind,st]['BM'] for st in STt]
    print(RdBM)
    if wind!=5:
        plt.bar(Brs[i], RdBM, width = barWidth, align='center', edgecolor ='grey', color='blue')
    else:
        plt.bar(Brs[i], RdBM, width = barWidth, align='center', edgecolor ='grey', label ='BM',color='blue')
    RdFI=[AvTasks[wind,st]['FI'] for st in STt]
    if wind!=5:
        plt.bar(Brs[i], RdFI, width = barWidth, align='center',bottom=RdBM, edgecolor ='grey',color='green')
    else:
    #print(RdFI)
        plt.bar(Brs[i], RdFI, width = barWidth, align='center',bottom=RdBM, edgecolor ='grey', label ='FI',color='green')
    RdFT=[AvTasks[wind,st]['FT'] for st in STt]
    if wind!=5:
        plt.bar(Brs[i], RdFT, width = barWidth, align='center',bottom=list(np.array(RdFI)+np.array(RdBM)), edgecolor ='grey',color='orange')
    else:
    #print(list(np.array(RdFI)+np.array(RdBM)))
        plt.bar(Brs[i], RdFT, width = barWidth, align='center',bottom=list(np.array(RdFI)+np.array(RdBM)), edgecolor ='grey', label ='FT',color='orange')
    i=i+1
plt.legend()
plt.xlabel('Plan Duration (Minute)')
plt.ylabel('Number of Subtasks (s)')
#plt.yscale('symlog')
#plt.xticks([r + barWidth for r in range(len(STt))],['0-20', '20-40', '40-60', '60-80'])

ax.set_xticklabels(('5','\n\n0-20', '10', '15','5','\n\n20-40', '10', '15','5','\n\n40-60', '10', '15','5','\n\n60-80', '10', '15'))#,ha='center')
# hide tick lines for x axis
ax.tick_params(axis='x', which='both',length=0)
# rotate labels with A


plt.grid( linestyle = '--', linewidth = 1)
#plt.savefig(f"./Results/FP_Runtime_Unit1.eps", bbox_inches='tight')
plt.show()
plt.clf() 








