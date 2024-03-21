#import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
with open(f'./160.csv', 'r') as file:
    t_s = []
    V_mV = []
    lines = file.readlines()
    for line in lines:
        if line[0] == 't':
            continue
        else:
            t_s.append(line[0:line.index(',')])
            V_mV.append(line[line.index(',')+1:-1])
for i in range(len(t_s)):
    t_s[i] = float(t_s[i])
    V_mV[i] = float(V_mV[i])
t = np.array(list(t_s))
vol = np.array(list(V_mV))
v = t
u = np.log(vol/12.2898)
k1, b1 = np.polyfit(v, u, 1)

x = np.array([0., 910])
mu = np.mean(u) # средее
mv = np.mean(v)
mv2 = np.mean(v**2) # средний квадрат
mu2 = np.mean(u**2)
muv = np.mean (u * v) # среднее от произведения
k = (muv - mu * mv) / (mv2 - mv**2)
b = mu - k * mv
N = len(v)
sigma_k = np.sqrt(1/(N-2) * ( (mu2 - mu**2)/(mv2 - mv**2) - k**2 ))
d = -k1 * (1200/1000000)*5.5/2*100
sd = (((30/1200)**2 + (0.5/5.5)**2)**(0.5))*d
x = np.array([0. , 550])
fig, ax = plt.subplots()
ax.errorbar(v, u , fmt = '.', linestyle=None, label='Экспериментальные точки')
ax.plot(x, k1 * x + b1, label='Линейная аппроксимация')
ax.set_title('Рис.4. Зависимость ln(U/U0) от t при давлении 160 торр')
ax.set_ylabel('ln(U/Uo)')
ax.set_xlabel('Время, с')
ax.legend()
plt.show()
print(k1, b1)
print(sigma_k)
print(d)
print(sd)