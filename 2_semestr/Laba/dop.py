import numpy
import numpy as np
from math import *
import matplotlib.pyplot as plt
x = numpy.array([44, 42, 41, 40, 39])
y = numpy.array([0.1, 0.3, 0.5, 0.7, 0.9])
a,b= numpy.polyfit(numpy.log(x),y,1)
print(a,b)
x1 = []
g = 38
for i in range(700):
    x1.append(g)
    g+=0.01
fib = plt.figure(figsize=(16,9))
ax1 = fib.add_subplot()
ax1.plot (x1, a*numpy.log(x1) + b ,  color = 'k')
ax1.errorbar(x,y, yerr = 0.01 , xerr = 0.8, linestyle = 'None', marker = 'o', label = 'Зависимость скорости полёта от расстояния до пистолета')
ax1.set_ylabel("Дальность стрельбы, s(м)")
ax1.set_xlabel("Скорость полёта, V(м/с)")
ax1.legend()
plt.show()