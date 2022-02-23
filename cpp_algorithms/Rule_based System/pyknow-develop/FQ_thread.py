from queue import Queue
from threading import Thread

# A thread that produces data
def producer(out_q):
    while True:
        # Produce some data
        a=input("Type number\n")
        if a=='q':
            break
        out_q.put(a)
    print('Stop sending')
   
# A thread that consumes data
def consumer(in_q):
    while True:
    # Get some data
        data = in_q.get()
        print(f"get {data}")
        if data=='1':
            break
        # Process the data
            print(data)
    print("Stop receiving")


# Create the shared queue and launch both threads
q = Queue()
t1 = Thread(target = consumer, args =(q, ))
t2 = Thread(target = producer, args =(q, ))
t1.start()
t2.start()

