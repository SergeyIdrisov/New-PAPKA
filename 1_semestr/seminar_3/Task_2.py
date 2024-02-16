from math import *
N=int(input())
res=[]
B=N
i=2
while i<=int(sqrt(B)):
    if N%i == 0:
        N = N // i
        res.append(i)
        i-=1
    i+=1
if N==1:
    print(*res)
else:
    res.append(N)
    print (*res)