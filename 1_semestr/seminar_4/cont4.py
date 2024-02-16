import numpy as np
import matplotlib.pyplot as plt

t0 = [30.75, 30.76, 30.76, 30.76] # время 20 колебаний, [с]
mean = np.mean(t0) # среднее
N = len(t0) # число опытов
sigma_t = np.sqrt( 1 / (N - 1) * np.sum( (t0 - mean)**2 ) )
l = 0.999 # м
m_0=0.8909
m_pr=0.0756
m_gr=0.3158
y=[67,61,55,49,43,37,31,25,19,13,7]
x_c0=0.2186
x_cm = []
res=[]
g=[]
a=0.2186
n = np.array([20,20,20,20,20,20,20,20,20,20,20])
t = np.array([32.16, 31.36,30.60,29.92,29.25,28.76,28.43,28.19,28.29,28.60,29.29])
T = np.array(t) / n
J = (m_0*(l**2))/12+m_0*(a**2)
for i in range(len(y)):
    x_cm.append((y[i]*m_gr*1.026268/100+m_0*x_c0)/(m_gr+m_0+m_pr))
    res.append(y[i] * 1.026268*10)
    g.append((4*(3.14**2)*(J+m_gr*((y[i]*1.026268/100)**2)))/((m_gr+m_0+m_pr)*x_cm[i]*(T[i]**2)))

print(res)


'''plt.figure(figsize=(8,6), dpi=100)
plt.ylabel("$T$, с")
plt.xlabel("$a$, м")
plt.xlim([0, 0.5])
plt.title('Рис.1. График зависимости периода $T$ от положения груза $a$')
plt.grid(True, linestyle="--")
plt.errorbar(x_cm, T,  fmt=".k", label="Экспериментальные точки")
plt.plot(x_cm, T, "--r", linewidth=1, label="Кусочно линейная интерполяция")
plt.plot([0.00,0.5], [1.53, 1.53], "--b", linewidth=1, label="Минимум")
plt.legend()'''
z=[]
for i in range(len(y)):
    z.append(y[i]**2)

v=[]
for i in range(len(x_cm)):
    v.append(x_cm[i]*(T[i]**2))
plt.figure(figsize=(16,9), dpi=100)
#plt.title('Рис.1. График зависимости периода $T$ от положения груза $a$')
#plt.grid(True, linestyle="--")
plt.plot(z, v)
plt.legend()
plt.show()
print ()

