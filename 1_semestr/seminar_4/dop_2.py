import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
x = [0,1,2,3,4]
y = [0,2,4,6,8]
plt.figure(figsize=(4,3), dpi=120)
plt.plot(x,y, 'b^--', label='2x')
plt.legend()
plt.show()