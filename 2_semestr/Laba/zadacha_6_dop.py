from tkinter import *
import matplotlib.pyplot as plt
from math import *
fib = plt.figure(figsize=(16,9))
ax1 = fib.add_subplot()
x1 = list(map(float, input().split()))
y1 = list(map(float, input().split()))
x2 = list(map(float, input().split()))
y2 = list(map(float, input().split()))
x3 = list(map(float, input().split()))
y3 = list(map(float, input().split()))

def a(x,y):
    Ex = 0
    Ey = 0
    Exy = 0
    Ex2 = 0
    sum = len(x)
    A = 0
    for i in range(sum):
        Ex += x[i]
        Ey += y[i]
        Exy += x[i] * y[i]
        Ex2 += x[i] ** 2
    a = (Ex * Ey - sum * Exy) / (Ex ** 2 - sum * Ex2)
    b = (Ex * Exy - Ex2 * Ey) / (Ex ** 2 - sum * Ex2)
    return a
def e(x,y):
    Ex = 0
    Ey = 0
    Exy = 0
    Ex2 = 0
    sum = len(x)
    A = 0
    for i in range(sum):
        Ex += x[i]
        Ey += y[i]
        Exy += x[i] * y[i]
        Ex2 += x[i] ** 2
    a = (Ex * Ey - sum * Exy) / (Ex ** 2 - sum * Ex2)
    b = (Ex * Exy - Ex2 * Ey) / (Ex ** 2 - sum * Ex2)
    for i in range(sum):
        y_sr = a * x[i] + b
        A += abs((y[i] - y_sr) / y[i])
    e = (A / sum) * 100
    return e
def b(x,y):
    Ex = 0
    Ey = 0
    Exy = 0
    Ex2 = 0
    sum = len(x)
    A = 0
    for i in range(sum):
        Ex += x[i]
        Ey += y[i]
        Exy += x[i] * y[i]
        Ex2 += x[i] ** 2
    a = (Ex * Ey - sum * Exy) / (Ex ** 2 - sum * Ex2)
    b = (Ex * Exy - Ex2 * Ey) / (Ex ** 2 - sum * Ex2)
    return b
ax1.errorbar (x1,y1, color = 'k', linestyle = 'None', marker = 'o')
x_vs = [0, x1[-1]+x1[0]/10]
y_vs = []
for i in range(2):
    y_vs.append(a(x1,y1)*x_vs[i]+b(x1,y1))
print(a(x1,y1),b(x1,y1),e(x1,y1))
ax1.errorbar(x_vs,y_vs, label = 'Медь')
ax1.errorbar (x2,y2, color = 'k', linestyle = 'None', marker = 'o')
x_vs = [0, x2[-1]+x2[0]/10]
y_vs = []
for i in range(2):
    y_vs.append(a(x2,y2)*x_vs[i]+b(x2,y2))
print(a(x2,y2),b(x2,y2))
ax1.errorbar(x_vs,y_vs, label = 'Железо')
ax1.errorbar (x3,y3, color = 'k', linestyle = 'None', marker = 'o')
x_vs = [0, x3[-1]+x3[0]/10]
y_vs = []
for i in range(2):
    y_vs.append(a(x3,y3)*x_vs[i]+b(x3,y3))

ax1.errorbar(x_vs,y_vs, label = 'Алюм-дюраль')
print(a(x3,y3),b(x3,y3))
plt.xlabel('n')
plt.ylabel('f, кГц')
plt.legend()
plt.show()
'''1 2 3 4 5 6 7
3.17256 6.422189 9.652789 12.86553 16.06601 19.27844 22.46672 
1 2 3 4 5 6 7
4.129924 8.281127 12.39818 16.53484 20.64631 24.77873 28.89545
1 2 3 4 5 6 7
4.23199 8.48224 12.7107 16.9378 21.1597 25.3666 29.5703
'''