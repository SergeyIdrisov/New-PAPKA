# импорт библиотек
import numpy as np # для обработки данных
import matplotlib.pyplot as plt # для построения графиков
t0 = [30.75, 30.76, 30.76, 30.76] # время 20 колебаний, [с]
mean = np.mean(t0) # среднее
N = len(t0) # число опытов
sigma_t = np.sqrt( 1 / (N - 1) * np.sum( (t0 - mean)**2 ) )
# тот же результат даёт встроенная функция
# np.std(t0, ddof=1)
print("t_mean = ", mean, "; sigma_t = %.3f" % sigma_t)
l = 0.999 # м
a = np.array([687.59956, 626.02348, 564.4474, 502.87131999999997, 441.29524, 379.71916, 318.14307999999994, 256.567, 194.99091999999996, 133.41484, 71.83876]) # [мм]
a = a / 1e3 # перевод в [м]
N = len(a) # число точек
n = np.array([20,20,20,20,20,20,20,20,20,20,20])
t = np.array([32.16, 31.36,30.60,29.92,29.25,28.76,28.43,28.19,28.29,28.60,29.29])
T = np.array(t) / n
print (T)
sigma_a = 0.5e-3
sigma_T = sigma_t / t * T
print(sigma_T)
gs = 4 * np.pi**2 * ( l**2 / 12 + a**2 ) / (a * T**2)
gm = np.mean(gs)
print(gs)
print("g_mean = %.3f" % gm)
sigma_gm = np.std(gs) / np.sqrt(N)
print("sigma_gm = %.3f" % sigma_gm)
plt.figure(figsize=(8,6), dpi=100) # размер графика
plt.ylabel("$T$, с") # подписи к осям
plt.xlabel("$a$, м")
plt.xlim([0, 0.7])
plt.title('Рис.1. График зависимости периода $T$ от положения груза $y$') # заголовок
plt.grid(True, linestyle="--") # пунктирная сетка
plt.errorbar(a, T, xerr=sigma_a, yerr=sigma_T, fmt=".k", label="Экспериментальные точки") #точки с погрешностями
plt.plot(a, T, "--r", linewidth=1, label="Кусочно линейная интерполяция") # интерполяция
plt.plot([0.00,0.7], [1.4095, 1.4095], "--b", linewidth=1, label="Минимум") # минимум
plt.legend() # легенда
plt.show()
u = T**2 * a
v = a**2
print("u = ", u, "\nv = ", v)
sigma_u = u * np.sqrt(4 * (sigma_T / T)**2 + (sigma_a/a)**2)
sigma_v = 2 * a * sigma_a
plt.plot(v, u, "+")
mu = np.mean(u) # средее
mv = np.mean(v)
mv2 = np.mean(v**2) # средний квадрат
mu2 = np.mean(u**2)
muv = np.mean (u * v) # среднее от произведения
k = (muv - mu * mv) / (mv2 - mv**2)
b = mu - k * mv
print("k = ", k, ", b = ", b)
np.polyfit(v, u, 1)
plt.figure(figsize=(8,6), dpi=100) # размер графика
plt.ylabel("$u=T^2 y$, $с^2 \cdot м$") # подписи к осям
plt.xlabel("$v=y^2$, $м^2$")
plt.title('Рис.2. Наилучшая прямая для линеаризованной зависимости $T(y)$') # заголовокграфика
plt.grid(True, linestyle="--") # сетка
plt.axis([0,0.5,0,2]) # масштабы осей
x = np.array([0., 1]) # две точки аппроксимирующей прямой
plt.plot(x, k * x + b, "-r",linewidth=1, label="Линейная аппроксимация $u = %.2f v + %.2f$"% (k, b)) # аппроксимация
plt.errorbar(v, u, xerr=sigma_a, yerr=sigma_T, fmt="ok", label="Экспериментальные точки",ms=3) # точки с погрешностями
plt.legend() # легенда
plt.show()
print(sigma_a,sigma_T)