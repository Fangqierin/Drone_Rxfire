from random import choice
import threading 
import zmq 
import time
from pyknow import *
from collections import defaultdict

class DetectFire(Fact):
    pass
class Grid(Fact):
    pass
class Phase2(Fact):
    pass
class Time(Fact):
    pass


class TaskGenerator(KnowledgeEngine):
    @DefFacts()
    def startup(self):
        for i in range(100):
            yield (Grid(id=i,state='UK',time=0))
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
        tasks[i]['FD']=t
    #############Enter Phase 2:
    @Rule(AS.f<<Phase2(L(-1))&FORALL(Grid(id=MATCH.i),Grid(id=MATCH.i,state=~L('UK')))&Time(MATCH.ct))
    def all_students_passed(self,ct,f):
        print(f"=====Get into Phase 2=====  at {ct}")
        self.modify(f,_0=ct)
    ######### Add task for fire tracking
    @Rule (Phase2(~L(-1))& AS.f<<Grid(state='N',id=MATCH.id,time=MATCH.t) & Phase2(MATCH.ct))
    def FT(self,f,id,t,ct):
        tasks[id]['FT']=max(t,ct) 
    ######### Add task for Intensity Monitor
    @Rule (Phase2(~L(-1))& AS.f<<Grid(state='B',id=MATCH.id,time=MATCH.t)& Phase2(MATCH.ct))
    def IM(self,f,id,t,ct):
        tasks[id]['IM']=max(t,ct)
    
Timer=0
tasks=defaultdict(dict)
t=TaskGenerator()
t.reset()
t.run()
#print(t.facts)
print(f"TASKS: {tasks}")
t.declare(DetectFire(id=1,yes=True, time=1))
t.declare(DetectFire(id=2,yes=False,time=3))
print("===2 run===")
t.run()
#print(t.facts)
print(f"TASKS: {tasks}")
t.declare(DetectFire(id=0,yes=True,time=4))
print("===3 run===")
t.run()
#print(t.facts)
print(f"TASKS: {tasks}")
t.declare(DetectFire(id=1,yes=False,time=5))
t.declare(DetectFire(id=2,yes=True,time=5))
print("===4 run===")
t.run()
#print(t.facts)
print(f"TASKS: {tasks}")
t.declare(DetectFire(id=2,yes=True,time=6))
t.declare(DetectFire(id=2,yes=False,time=8))
print("===5 run===")
t.run()
#print(t.facts)
print(f"TASKS: {tasks}")
print(t.facts)








