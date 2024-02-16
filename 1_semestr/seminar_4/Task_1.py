import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
y_znach=[225,300,350,390,450,535,585,640,675,740]
x_znach=[109.94, 147.11,173.25,192.06,222.28,263.53,287.87,316.15,332.15,365.3]

fig = plt.figure(figsize = (16,9), dpi=100)
ax1 = fig.add_subplot(111)

x = [109.94, 365.33]
y = np.interp(x, x_znach, y_znach)

ax1.errorbar(x_znach, y_znach, yerr=1.25, xerr = 0.75, color = 'k', linestyle = 'None')

ax1.plot(x,y, 'r', label = 'R(Om)')
plt.xlabel('I(mA)')
plt.ylabel('V(mV)')
ax1.legend()
ax1.grid()
fig.show()