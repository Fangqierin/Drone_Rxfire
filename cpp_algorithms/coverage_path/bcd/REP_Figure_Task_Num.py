"""
Some helper functions that are often used.
"""
import numpy as np
import matplotlib.pyplot as plt
from collections import defaultdict
import math
#from .constants import OB, NO
import re

STt=[1,20,40,60]
for Unit in [1,2]:
    if Unit==1:
        Winds=[5,10,15]#,20,25]
    else:
        Winds=[5,10,15]
    # if Unit==3:
    #     Winds=[15,20,25]
    STtime=60
    StReward=defaultdict(dict)
    StMiss=defaultdict(dict)
    StRuntime=defaultdict(dict)
    for wind in Winds:
        for ST in STt:
            STtime=ST
            # if Unit==3:
            #     if  STtime==1 or wind in [20,25]:
            #         Simfile=f"./Results/Simple222_{Unit}_{wind}_{STtime}"
            #     else:
            #         Simfile=f"./Results/Simple14_{Unit}_{wind}_{STtime}"
            # if Unit!=3 and wind in [20,25]:
            #     Simfile=f"./Results/Simple222_{Unit}_{wind}_{STtime}"
            #else:
            Simfile=f"./Results/REP_Simple23_{Unit}_{wind}_{STtime}"
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
    
    # for i in range(4):
    #     Brs.append([x + barWidth for x in Brs[-1]])
    # i=0
    # colors=['plum','orange','lightblue']
    # for wind in Winds:
    #     for fpm in [5]:
    #     #for tav in TAKM:
    #         print(f"{wind} ")
    #         Rd=[np.mean([StRuntime[wind,st][2,2,fpm,k] for k in range(10)])  for st in STt]
    #         Rdst=np.array([np.std([StRuntime[wind,st][2,2,fpm,k] for k in range(10)]) for st in STt])
    #         Rder=conf_co*(Rdst/math.sqrt(10))
    #         #Rd=[StMiss[wind,st][2,1,fpm] for st in STt]
    #         #print(f" {wind} {st} {tav} {} {StMiss[wind,st][tav,1,0]}")
    #         #plt.bar(Brs[i], Rd, width = barWidth, edgecolor ='grey', label =Label[fpm])
    #         plt.bar(Brs[i], Rd, width = barWidth, yerr=Rder, align='center',capsize=5, edgecolor ='grey', label =Label[i],color=colors[i])#,color=plt.rcParams['axes.color_cycle'][2])
    #         i=i+1
    # plt.legend()
    # plt.rcParams["image.cmap"] = "Accent"
    # plt.rcParams['axes.prop_cycle'] = plt.cycler(color=plt.cm.Set1.colors)
    #
    # plt.xlabel('Plan Duration (Minute)')
    # plt.ylabel('Running Time of UTA-DFP (s)')
    # #plt.rcParams['axes.color_cycle'] 
    # #plt.yscale('symlog')
    # plt.xticks([r + barWidth for r in range(len(STt))],['0-20', '20-40', '40-60', '60-80'])
    # plt.grid( linestyle = '--', linewidth = 1)
    # plt.savefig(f"./Results/FP_Runtime_{Unit}.eps", bbox_inches='tight')
    # plt.show()
    # plt.clf() 
    
    ####################################### Get the Total tasks in different seeds 
    SumTasks=defaultdict(dict)
    AvTasks=defaultdict(dict)
    for wind in Winds:
        for ST in STt:
            STtime=ST
            if Unit==3:
                if  STtime==1 or wind in [20,25]:
                    Simfile=f"./Results/Simple222_{Unit}_{wind}_{STtime}"
                else:
                    Simfile=f"./Results/Simple14_{Unit}_{wind}_{STtime}"
            if Unit!=3 and  wind in [20,25]:
                Simfile=f"./Results/Simple222_{Unit}_{wind}_{STtime}"
            else:
                Simfile=f"./Results/Simple14_{Unit}_{wind}_{STtime}"
            log=open(Simfile,"r")
            filedata = log.readlines()
            SumRreward=defaultdict(dict)
            SumMiss=defaultdict(dict)
            Runtime=defaultdict(dict)
            for line in filedata:
                if re.match(r'Tasks 2 1 0', line):
                    allitm=line[:-3].split('; ') 
                    #print(f"{wind} {STtime} allitm {allitm}")
                    #print(allitm)
                    itms=allitm[0].split(' ')
                    minus=int(allitm[-1].split(' ')[-1]) # For 1200 extra added tasks 
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
                print(f"seeee",(wind,ST),tak, int(np.mean([SumTasks[(wind,ST,s)][tak] for s in range(10)])))
    print(SumTasks)
    #print(SumRreward)
    #fig, ax = plt.subplots(figsize=(10, 6))
    fig,ax = plt.subplots(figsize =(6, 5))
    plt.rcParams.update({'font.size': 9})
    
    if Unit==10:
        tick_labels_1 = ['5']* len(STt)
        tick_labels_2 = ['10'] * len(STt)
        tick_labels_3 = ['15\n0-20 min','15\n20-40 min', '15\n40-60 min','15\n60-80 min']
        tick_labels_4 = ['20'] * len(STt)
        tick_labels_5 = ['25'] * len(STt)
        # #print([x - 0.4, x - 0.1, x + 0.2])
        # all_x = np.concatenate([x - 0.4, x - 0.1, x + 0.2])
        # #print(all_x)
        labels=tick_labels_1+tick_labels_2+tick_labels_3+tick_labels_4+tick_labels_5
        #labels=['5', '10', '15', '5', '10', '15','5', '10', '15','5', '10', '15']
        
        #labels=['5', '10\n0-20', '15','5', '10\n20-40', '15','5', '10\n40-60', '15','5', '10\n60-80', '15']
        #Label=[ '5 MPH wind', '10 MPH wind', '15 MPH wind']
        barWidth = 0.15
        Brs=[np.arange(len(STt))]
        for i in range(len(Winds)-1):
            Brs.append([x + barWidth+0.03 for x in Brs[-1]])

    else: 
        tick_labels_1 = ['5']* len(STt)
        #print(tick_labels_1)
        tick_labels_2 = ['10\n0-20 min','10\n20-40 min', '10\n40-60 min','10\n60-80 min']
        tick_labels_3 = ['15'] * len(STt)
        # #print([x - 0.4, x - 0.1, x + 0.2])
        # all_x = np.concatenate([x - 0.4, x - 0.1, x + 0.2])
        # #print(all_x)
        labels=tick_labels_1+tick_labels_2+tick_labels_3
        #labels=['5', '10', '15', '5', '10', '15','5', '10', '15','5', '10', '15']
        
        #labels=['5', '10\n0-20', '15','5', '10\n20-40', '15','5', '10\n40-60', '15','5', '10\n60-80', '15']
        Label=[ '5 MPH wind', '10 MPH wind', '15 MPH wind']
        barWidth = 0.24
        Brs=[np.arange(len(STt))]
        for i in range(len(Winds)-1):
            Brs.append([x + barWidth+0.04 for x in Brs[-1]])
        
    Colors=['blue','green','orange']
    
    #print(np.concatenate([np.array([AvTasks[wind,st]['BM'] for st in STt]) for  wind in Winds]))
    #print(np.concatenate([np.array(Brs[i]) for i in range(len(Brs))]))
    i=0
    Rdlist=[]
    for tak in ['BM','FI','FT']:
        print(wind,AvTasks)
        print(tak,[[AvTasks[wind,st][tak] for st in STt] for  wind in Winds])
        Rd=np.concatenate([np.array([AvTasks[wind,st][tak] for st in STt]) for  wind in Winds])
        Rdlist.append(Rd)
        if i==0:
            #print(np.concatenate([np.array(Brs[n]) for n in range(len(Brs))]),Rd)
            ax.bar(np.concatenate([np.array(Brs[n]) for n in range(len(Brs))]), Rd, width = barWidth, align='center', edgecolor ='grey', color=Colors[i],tick_label=labels,label =tak)
        if i!=0:
            botV=np.array(Rdlist[0])
            for k in Rdlist[1:-1]:
                #print(tak,k)
                botV=botV+np.array(k)
            ax.bar(np.concatenate([np.array(Brs[n]) for n in range(len(Brs))]), Rd, width = barWidth,bottom=list(botV), align='center', edgecolor ='grey', color=Colors[i],label=tak )
        i=i+1
    plt.legend()
    #plt.xlabel('Plan Duration (Minute)')
    plt.ylabel('Number of Subtasks')
    #plt.yscale('symlog')
    plt.figtext(0.01, 0.034, 'Plan Duration:', fontsize = 10)
    plt.figtext(0.01, 0.07, 'Wind (mph):', fontsize = 10)
    plt.grid( linestyle = '--', linewidth = 1)
    plt.tight_layout()
    plt.savefig(f"./Results/TaskNum_{Unit}.eps", bbox_inches='tight')
    plt.show()
Unit=3
STtime=60
StReward=defaultdict(dict)
StMiss=defaultdict(dict)
StRuntime=defaultdict(dict)
####################################### Get the Total tasks in different seeds 
SumTasks=defaultdict(dict)
AvTasks=defaultdict(dict)
for wind in Winds:
    for ST in STt:
        STtime=ST
        if Unit==3:
            if  STtime==1 or wind in [20,25]:
                Simfile=f"./Results/Simple222_{Unit}_{wind}_{STtime}"
                
        else:
            Simfile=f"./Results/Simple14_{Unit}_{wind}_{STtime}"
        log=open(Simfile,"r")
        filedata = log.readlines()
        SumRreward=defaultdict(dict)
        SumMiss=defaultdict(dict)
        Runtime=defaultdict(dict)
        for line in filedata:
            if re.match(r'Tasks 1 2 0', line):
                allitm=line[:-3].split('; ') 
                print(f"{wind} {STtime} allitm {allitm}")
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
            #print(f"seeee",(wind,ST),tak, int(np.mean([SumTasks[(wind,ST,s)][tak] for s in range(10)])))
fig,ax = plt.subplots(figsize =(6, 5))

plt.rcParams.update({'font.size': 9})

tick_labels_1 = ['5']* len(STt)
tick_labels_2 = ['10\n0-20 min','10\n20-40 min', '10\n40-60 min','10\n60-80 min']
tick_labels_3 = ['15'] * len(STt)

labels=tick_labels_1+tick_labels_2+tick_labels_3
Label=[ '5 MPH wind', '10 MPH wind', '15 MPH wind']
barWidth = 0.24
Brs=[np.arange(len(STt))]
for i in range(len(Winds)-1):
    Brs.append([x + barWidth+0.04 for x in Brs[-1]])
Colors=['blue','green','orange']
i=0
Rdlist=[]
for tak in ['BM','FI','FT']:
    print(tak,[[AvTasks[wind,st][tak] for st in STt] for  wind in Winds])
    Rd=np.concatenate([np.array([AvTasks[wind,st][tak] for st in STt]) for  wind in Winds])
    Rdlist.append(Rd)
    if i==0:
        print(np.concatenate([np.array(Brs[n]) for n in range(len(Brs))]),Rd)
        ax.bar(np.concatenate([np.array(Brs[n]) for n in range(len(Brs))]), Rd, width = barWidth, align='center', edgecolor ='grey', color=Colors[i],tick_label=labels,label =tak)
    if i!=0:
        botV=np.array(Rdlist[0])
        for k in Rdlist[1:-1]:
            #print(tak,k)
            botV=botV+np.array(k)
        ax.bar(np.concatenate([np.array(Brs[n]) for n in range(len(Brs))]), Rd, width = barWidth,bottom=list(botV), align='center', edgecolor ='grey', color=Colors[i],label=tak )
    i=i+1
plt.legend()
#plt.xlabel('Plan Duration (Minute)')
plt.ylabel('Number of Subtasks')
#plt.yscale('symlog')
plt.figtext(0.01, 0.034, 'Plan Duration:', fontsize = 10)
plt.figtext(0.01, 0.07, 'Wind (mph):', fontsize = 10)
plt.grid( linestyle = '--', linewidth = 1)
plt.tight_layout()
plt.savefig(f"./Results/TaskNum_{Unit}.eps", bbox_inches='tight')
plt.show()





