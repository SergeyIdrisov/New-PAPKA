
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
N=int(input())
fibs = [0 for i in range(N)]
print(fib(N,fibs))