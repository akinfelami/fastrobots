from multiprocessing.connection import Listener, Client, CONNECTION_TIMEOUT
import time
import numpy as np

address = ('localhost',600)
print(CONNECTION_TIMEOUT)
total = 1000000
times = np.zeros((total,1))
for i in range(0,total):
    start_time = time.time()
    Client(address, b'asdasdasd')
    times[i] = time.time() - start_time

print("Total Time: ", np.sum(times))
print("Avg. Time : ", np.mean(times))

    