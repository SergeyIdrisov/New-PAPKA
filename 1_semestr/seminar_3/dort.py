import timeit
import sys
import matplotlib
import matplotlib.pyplot as plt

sys.setrecursionlimit(100000)
def _y(N):
    result = timeit.default_timer()
    fib1 = 1
    fib2 = 1

    n = N

    i = 0
    while i < n - 2:
        fib_sum = fib1 + fib2
        fib1 = fib2
        fib2 = fib_sum
        i = i + 1
    end = timeit.default_timer() - result
    return end
fib = plt.figure(figsize=(9,16))
ax1 = fib.add_subplot()
x = []
y = []
for i in range(2,10000):
    x.append(i)
    y.append(_y(i))
ax1.plot(x,y,marker = 'o')
plt.show()