from random import choice
import threading 
import zmq 
import time
from pyknow import *
from collections import defaultdict
import subprocess
from FQ_Task_Generator import TG, DetectFire, tasks, Timer
from FQ_GenTask import GetEFA
#TG=TaskGenerator()
#p=subprocess.Popen(f"pwd", )


p=subprocess.Popen(f"python3 ./FQ_Task_Generator.py", shell=True)
p.wait()

print(f"TASKS: {tasks}")
TG.declare(DetectFire(id=1,yes=True, time=1))
TG.declare(DetectFire(id=2,yes=False,time=3))
print("===2 run===")
TG.run()
print(TG.facts)
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
print(TG.facts)
print(f"TASKS: {tasks}")
TG.declare(DetectFire(id=2,yes=True,time=6))
TG.declare(DetectFire(id=2,yes=False,time=8))
print("===5 run===")
TG.run()
print(TG.facts)
print(f"TASKS: {tasks}")
print(TG.facts)

if __name__ == "__main__":
    Missions=defaultdict(dict)  #  Missions: period, priority, 
    Missions['BM']=[ 10, 1]  #Other area 
    Missions['FI']=[ 5, 1]  #track the fire 
    Missions['FT']=[ 3, 3]  # track the fire perimeter and the risky area (arrival within 10 min)
    Missions['FL']=[ 3, 2]  # FL is tracking the fire perimeter.
    Missions['FD']=[ 2, 3]
    ################ Get tasks.
    dir='/home/fangqiliu/eclipse-workspace_Python/Drone_path/CoveragePathPlanning-master/farsite/'
    foldername='FQ_burn'
    BunsiteBound=(701235.0,4308525.0,704355.0,4311675.0)
    BunsiteBound=(702374.0,4309425.0,703700.0,4310900.0)
    Bursite=(702460.0,4309700.0,703540.0,4310820.0 )
    init=120
    end=150
    Res=10
    EFA,EFAdict,Primdict =GetEFA(init,end,Bursite,dir,foldername, Res=Res)
    EFA,EFAdict,Primdict =GetEFA(init,end,Bursite,dir,foldername, Res=Res)

    










