#!/usr/bin/env python
# coding: utf-8

# In[2]:


def fact(N, depth = 0):
    if N == 1:
        return 1
    print(depth) # прямой ход рекурсии 
    res = N*fact(N-1, depth+1) # рекурсивный вызов
    print(f'on step {depth} fact = {res}') # обратный ход рекурсии
    return res

fact(10)


# In[27]:


# Task 1 

# F(n) = F(n-1) + F(n-2), F(0) = 1, F(1) = 1


def fib(N, fibs):
    if N == 1:
        fibs[0] = 0
        return fibs[0]
    if N == 2:
        fibs[1] = 1
        return fibs[1]
    
    if fibs[N-1] != 0:
        return fibs[N-1]

    fibs[N-1] = fib(N - 1, fibs) + fib(N - 2, fibs)
    print(fibs)
    return fibs[N-1]

    
N = 10
fibs = [0 for i in range(N)]
print(fib(N, fibs))
# 0 1 1 2 3 5 8




# In[29]:


# динамическое программирование
N = int(input())
fibs = [0 for i in range(N)]

fibs[0] = 0
fibs[1] = 1

for i in range(2,N):
    fibs[i] = fibs[i-1] + fibs[i-2]

print(fibs)


# In[51]:


# Task 2

import numpy as np
# pip install numpy
N = int(input())

sqr = np.sqrt(N)
#print(int(sqr))
dividers = []
i = 2

while i < (int(sqr)+1):
    if N % i == 0:
        N = N // i
        dividers.append(i)
        i -= 1
    i += 1
print(dividers)
# доделать для простых множителей больше корня


# In[3]:


def tri(N, i):
    if N == 0:
        return
    print('.'*i)
    tri(N-1,i+1)
    print('.'*i)
N = int(input())
if N % 2 == 1:
    tri(N//2  + 1, 1)
else:
    tri(N//2, 1)
    
# модифицировать на чет/нечет либо написать через цикл


# In[1]:


N = int(input())
M = int(input())

# создать матрицу NxM
B = []
for i in range(N):
    A = [0 for j in range(M)]
    B.append(A)
print(*B, sep = '\n')


# In[13]:


import numpy as np
tx = [1,2,3,4,5]
ty = [2*i + 1 for i in tx]
x = np.array(tx)
y = np.array(ty)

#print(y)

x_mean = np.mean(x)
y_mean = np.mean(y)

numerator = (x - x_mean)*y
denominator = (x - x_mean)**2
a = np.sum(numerator)/np.sum(denominator)
print(a)
b = y_mean - a*x_mean
print(b)

#на дом: посчитать погрешности a и b по формулам


# ![image.png](attachment:image.png)

# In[18]:


nums = [i**3 + 10 for i in range(10)]
nums = np.array(nums)
print(nums - 10)


# In[ ]:




