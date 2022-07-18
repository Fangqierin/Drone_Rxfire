from random import choice
import threading 
import zmq 
import time
from pyknow import *
from collections import defaultdict
import subprocess
from FQ_Task_Generator import TG, DetectFire, tasks, Timer

#TG=TaskGenerator()
#p=subprocess.Popen(f"pwd", )

p=subprocess.Popen(f"python3 ./FQ_Task_Generator.py", shell=True)
p.wait()

print(f"TASKS: {tasks}")
TG.declare(DetectFire(id=1,yes=True, time=1))
TG.declare(DetectFire(id=2,yes=False,time=3))
print("===2 run===")
TG.run()
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

while True:
    pass