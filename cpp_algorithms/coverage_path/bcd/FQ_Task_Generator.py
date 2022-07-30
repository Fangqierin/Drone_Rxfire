from random import choice
import threading 
import zmq 
import time
from pyknow import *
from collections import defaultdict
#import sched
class DetectFire(Fact):
    pass
class Grid(Fact):
    pass
class Phase2(Fact):
    pass
class Time(Fact):
    pass
class Theta(Fact):
    pass
class Mission(Fact):
    pass
class TaskManager():
    def __init__(self, Missions):
        self.TG=TaskGenerator()
        self.TG.reset()
        # print(Missions)
        # for key, st in Missions.items():
        #     self.TG.declare(Mission(name=key,period=st[0]))
        #yield (Mission(BM=10, FI=5, FT=3, FD=5, step=3))
        #yield(Theta(v=9, plan=20))
        #yield (Time(Timer)
        self.Missions=Missions
        self.Mission_dict=defaultdict 
    def reset(self):
        self.TG.rest()
        tasks=defaultdict(dict)
    
    def RefineTask(self):
        pass
    
    def DeclareGrid(self, EFAM, init=1):
        rows,columns=EFAM.shape
        for x in range(rows):
            for y in range(columns):
                if EFAM[x,y]<=1:
                    st='B'
                else:
                    st='N'
                self.TG.declare(Grid(id=(x,y),state=st, EFA=EFAM[x,y],time=0))
        #TG.declare(Phase2((1)))
        self.TG.run()
        self.cur_tasks=tasks
        print(f"see tasks {tasks}")
        return tasks
        #print(tasks)  
    def UpdateEFA(self, EFAM, init=10):
        rows,columns=EFAM.shape
        for x in range(rows):
            for y in range(columns):
                if EFAM[x,y]<=1:
                    st='B'
                else:
                    st='N'
                self.TG.declare(Grid(id=(x,y),state=st, EFA=EFAM[x,y],time=0))
        #TG.declare(Phase2((1)))
        self.TG.run()
        self.cur_tasks=tasks
        print(f"see tasks {tasks}")
        
        
class TaskGenerator(KnowledgeEngine):
    @DefFacts()
    def startup(self):
        # for i in range(3):
        #     yield (Grid(id=i,state='UK',EFA=0))
        yield (Mission(BM=10, FI=5, FT=3, FD=5, step=3))
        yield(Theta(v=9, plan=20))
        yield (Time(Timer))
        yield(Phase2(-1))
    ########## For maintain a timer 
    @Rule(DetectFire(time=MATCH.t,id=MATCH.id), AS.f<< Time(MATCH.ct),salience=1)
    def CheckTime(self,t,f,ct):
        if ct<t:
            self.modify(f,_0=t)
    ########## For reduce redundancy
    @Rule(AS.f<<DetectFire(id=MATCH.id,yes=True) & Grid(id=MATCH.id, state='B'),salience=1)
    def Remove_Dup(self,f,id):
        self.retract(f)
    @Rule(AS.f<<DetectFire(id=MATCH.id,yes=False) & (Grid(id=MATCH.id, state='N')|Grid(id=MATCH.id, state='O')),salience=1)
    def Remove_Dup(self,f,id):
        self.retract(f)
    ##############ChangeState
    @Rule (AS.f1<<DetectFire(id=MATCH.id, yes=False,time=MATCH.t) & AS.f<<(Grid(state='UK', id=MATCH.id)))
    def ShiftUKtoN(self,f,f1,id,t):
        self.retract(f1)
        self.modify(f,state='N',time=t)
        tasks[id].pop('FD', None)
    @Rule (AS.f1<<DetectFire(id=MATCH.id, yes=True,time=MATCH.t) & AS.f<<(Grid(state='UK', id=MATCH.id)))
    def ShiftUKtoB(self,f,f1,id,t):
        self.retract(f1)
        self.modify(f,state='B',time=t)
        tasks[id].pop('FD', None)
    @Rule (AS.f1<<DetectFire(id=MATCH.id, yes=False,time=MATCH.t) & AS.f<<(Grid(state='B', id=MATCH.id)))
    def ShiftBtoO(self,f,f1,id,t):
        self.retract(f1)
        self.modify(f,state='O',time=t)
        tasks[id].pop('IM', None)
    @Rule (AS.f1<<DetectFire(id=MATCH.id, yes=True,time=MATCH.t) & AS.f<<(Grid(state='N', id=MATCH.id)))
    def ShiftNtoB(self,f,f1,id,t):
        self.retract(f1)
        self.modify(f,state='B',time=t)
        tasks[id].pop('FD', None)
        tasks[id].pop('FT', None)
    ############ Add task Fire detection
    @Rule (AS.f<<Grid(state='UK',id=MATCH.i,time=MATCH.t))
    def FD(self,f,i,t):
        tasks[i]['FD']=(t,-1)
    #############Enter Phase 2:
    @Rule(AS.f<<Phase2(L(-1))&FORALL(Grid(id=MATCH.i),Grid(id=MATCH.i,state=~L('UK')))&Time(MATCH.ct))
    def all_Grid_passed(self,ct,f):
        print(f"=====Get into Phase 2=====  at {ct}")
        self.modify(f,_0=ct)
    ######### Add task for fire tracking
    # @Rule (Phase2(~L(-1))& Mission(step=MATCH.step, BM=MATCH.BM, FT=MATCH.FT)&Theta(v=MATCH.v, plan=MATCH.plan) & AS.f<<Grid(state='N',id=MATCH.id, time=MATCH.t, EFA=MATCH.EFA) & Phase2(MATCH.ct))
    # def FT(self,f,id,t,EFA,ct,v, plan,step, BM, FT ):
    #     if max(t,ct)>=EFA-v:
    #         #print(f" see {step} {BM} {FT}")
    #         tasks[id]['FT']=(max(t,ct),-1)
    #     elif EFA==255 or EFA-v>=ct+plan:
    #         tasks[id]['BM']=(max(t,ct), -1)
    #     else: 
    #         if EFA-v-(max(t,ct))>BM:
    #             dd=(EFA-v-(max(t,ct)))//BM*BM+max(t,ct)
    #             tasks[id]['BM']=(max(t,ct), dd)
    #         #tmp=min(step,FT)
    #         st=EFA//step*step-v
    #         tasks[id]['FT']=(st, -1)
    #         #print(f"EFA {EFA} {EFA-v} {plan} ")
    #         print(f"add task {tasks[id]}")
    #     #print(f"Generate task FT at grid {id}")
    @Rule (Phase2(~L(-1))&Theta(v=MATCH.v, plan=MATCH.plan) & AS.f<<Grid(state='N',id=MATCH.id, time=MATCH.t, EFA=MATCH.EFA) & Phase2(MATCH.ct))
    def FT(self,f,id,t,EFA,ct,v, plan):
        if max(t,ct)>=EFA-v:
            tasks[id]['FT']=(max(t,ct),-1)
        elif EFA==255 or EFA-v>=ct+plan:
            tasks[id]['BM']=(max(t,ct), -1)
        else: 
            tasks[id]['BM']=(max(t,ct), EFA-v)
            tasks[id]['FT']=(EFA-v, -1)
            print(f"add task {tasks[id]}")
    ######### Add task for Intensity Monitor
    @Rule (Phase2(~L(-1))& AS.f<<Grid(state='B',id=MATCH.id,time=MATCH.t)& Phase2(MATCH.ct))
    def IM(self,f,id,t,ct):
        tasks[id]['FI']=(max(t,ct),-1)
Timer=0
tasks=defaultdict(dict)
Try=2
# TG=TaskGenerator()
# #TG=TaskGenerator(Timer,[])
# TG.reset()
# TG.run()
# #print(TG.facts)
# print(f"TASKS: {tasks}")
# TG.declare(DetectFire(id=1,yes=True, time=1))
# TG.declare(DetectFire(id=2,yes=False,time=3))
# print("===2 run===")
# TG.run()
'''
#print(TG.facts)
print(f"TASKS: {tasks}")
TG.declare(DetectFire(id=0,yes=True,time=4))
print("===3 run===")
TG.run()
#print(TG.facts)
print(f"TASKS: {tasks}")
TG.declare(DetectFire(id=1,yes=False,time=5))
TG.declare(DetectFire(id=2,yes=True,time=5))
print("===4 run===")
TG.run()
#print(TG.facts)
print(f"TASKS: {tasks}")
TG.declare(DetectFire(id=2,yes=True,time=6))
TG.declare(DetectFire(id=2,yes=False,time=8))
print("===5 run===")
TG.run()
#print(TG.facts)
print(f"TASKS: {tasks}")
print(TG.facts)

'''



