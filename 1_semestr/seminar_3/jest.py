import timeit
import sys
import matplotlib
import matplotlib.pyplot as plt

sys.setrecursionlimit(100000)
def _y(N):
    result = timeit.default_timer()

    def fib(N,fibs):

        if N==0:
            fibs[0]=0
            return 0

        if N == 1:
            fibs[1]=1
            return 1

        if fibs[N-1]!= 0:
            return fibs[N-1]

        fibs[N-1] = fib(N-1,fibs)+fib(N-2,fibs)
        return(fibs[N-1])
    fibs = [0 for i in range(N)]
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
