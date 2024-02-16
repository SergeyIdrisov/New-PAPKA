import matplotlib.pyplot as plt
import pandas as pd
fib = plt.figure(figsize=(16,9))
ax1 = fib.add_subplot(111)
close = list(map(int, pd.read_csv('BTC_data.csv')['close']))

data = []
for i in range(len(close)):
    b = list(pd.read_csv('BTC_data.csv')['time'])[i]
    data.append(b[0:10])
ax1.plot(data,close, 'k')

oboz=[]
for i in range(len(data)):
    if i%145 == 0:
        oboz.append(i)

ax1.set_xticks([oboz[i] for i in range(len(oboz))])
plt.show()