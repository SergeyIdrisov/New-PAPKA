import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
dan1 =list(pd.read_csv('iris_data.csv')['Species'])
fib = plt.figure(figsize=(9,16))
ax1 = fib.add_subplot(211)
ax2 = fib.add_subplot(212)
name = []
amount = []
b=[]
for i in range(len(dan1)):
    if b != dan1[i]:
        amount.append(dan1.count(dan1[i]))
        name.append(dan1[i])
        b=dan1[i]
ax1.pie([amount[i] for i in range(len(amount))], labels=[name[i] for i in range(len(name))])
ax1.set_title('Species')

dan2 = list(pd.read_csv('iris_data.csv')['m'])
a,b,c = 0,0,0
for i in range(len(dan2)):
    if dan2[i] <= 1.2:
        a +=1
    elif dan2[i] > 1.2 and dan2[i]< 1.5:
        b+=1
    else:
        c+=1
ax2.pie([a,b,c], labels=['<=1.2(cm)','>=1.2(cm) and < 1.5(cm)', '>=1.5(cm)'])
ax2.set_title('PetalLengthCm')
plt.show()