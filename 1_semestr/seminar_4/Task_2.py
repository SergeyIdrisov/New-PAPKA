import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
fig = plt.figure(figsize=(16,8))
ax1 = fig.add_subplot(211)
ax2 = fig.add_subplot(212)
dan1 = np.random.normal(0,10,1000)
ax1.hist(dan1,40)
ax1.grid()
dan2 = np.random.normal(0,10,1000000)
ax2.hist(dan2,40)
ax2.grid()
fig.show()

