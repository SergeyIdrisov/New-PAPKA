import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

fig = plt.figure(figsize=(16, 9))  # создали рисунок/Figure Fig пропорциями 16:9
ax1 = fig.add_subplot(
    211)  # создали Axes (подграфик) ax1 в серии из 2 графиков, поставили на позицию [1,1] -- левый верхний угол
ax2 = fig.add_subplot(
    212)  # создали Axes ax2 в серии из 2 графиков, поставили на позицию [1,2] -- первый график во второй "строке" графиков

# сгенерируем данные для какой-нибудь гистограммы
values = np.random.normal(0, 10, 1000)

# строим гистограмму с 50 блоками
ax1.hist(values, 50)
ax1.grid()  # делаем сетку на графике ax1

x = [i for i in range(50)]
y = [j ** 1.5 for j in x]

ax2.plot(y, x, 'b.', label='blue dots')
ax2.plot(x, y, 'r--', label='red dashed line')
ax2.set_title('second graph')  # здесь название функции немного отличается от случая, когда мы вызывали напрямую из plt!

ax2.grid()  # делаем сетку на графике ax2
ax2.legend()  # делаем легенду на графике ax2

fig.show()