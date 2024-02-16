import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from math import *
fib = plt.figure(figsize=(16,9))
ax1 = fib.add_subplot()
x1 = list(map(float, pd.read_csv('iris_data.csv')['SepalLengthCm']))
y1 = list(map(float, pd.read_csv('iris_data.csv')['SepalWidthCm']))
x2 = list(map(float, pd.read_csv('iris_data.csv')['m']))
y2 = list(map(float, pd.read_csv('iris_data.csv')['PetCalWidthm']))

def MNK1(x,y):
    xy = 0
    x2 = 0
    y_ = 0
    x_ = 0
    for i in range(len(x)):
        xy += x[i] * y[i]
        x_ += x[i]
        y_ += y[i]
        x2 += x[i] * x[i]
    xy /= len(x)
    x_ /= len(x)
    y_ /= len(x)
    x2 /= len(x)
    b = (xy - x_ * y_) / (x2 - x_ ** 2)
    a = y_ - b * x_
    return b
def MNK2(x,y):
    xy = 0
    x2 = 0
    y_ = 0
    x_ = 0
    for i in range(len(x)):
        xy += x[i] * y[i]
        x_ += x[i]
        y_ += y[i]
        x2 += x[i] * x[i]
    xy /= len(x)
    x_ /= len(x)
    y_ /= len(x)
    x2 /= len(x)
    b = (xy - x_ * y_) / (x2 - x_ ** 2)
    a = y_ - b * x_
    return a

x=[]
y=[]
x.append(MNK1(x1,y1))
x.append(MNK2(x1,y1))
ax1.errorbar (x1,y1, yerr = 0.05, xerr = 0.03, color = 'k', linestyle = 'None', label = 'Sepal')
for i in range(len(x1)):
    b =x[-1] + x[0]*x1[i]
    y.append(b)
ax1.plot(x1,y,'r')

ax1.errorbar(x2,y2, yerr = 0.05, xerr = 0.03, color = 'y', linestyle = 'None', label='Petal')
x = []
y=[]
x.append(MNK1(x2,y2))
x.append(MNK2(x2,y2))
for i in range(len(x1)):
    b =x[-1] + x[0]*x2[i]
    y.append(b)
ax1.plot(x2,y,'b')

fib.legend()
plt.show()


