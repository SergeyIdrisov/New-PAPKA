from math import *
N=list(map(int, input().split()))
def mnoj(B):
    res = []
    C = B
    i = 2
    while i <= int(sqrt(C)):
        if B % i == 0:
            B = B // i
            res.append(i)
            i -= 1
        i += 1
    if B == 1:
        return (res)
    else:
        res.append(B)
        return(res)
A=mnoj(N[0])
B=mnoj(N[-1])
res = 1
if N[0]>N[-1]:
    for i in range(N[0]+1):
        if A.count(i) == 0 or B.count(i) == 0:
            continue
        elif  A.count(i)<B.count(i):
            res*=i**A.count(i)
        elif A.count(i)>=B.count(i):
            res *= i**B.count(i)
elif N[0]<=N[-1]:
    for i in range(N[-1]+1):
        if A.count(i) == 0 or B.count(i) == 0:
            continue
        elif  A.count(i)<B.count(i):
            res*=i**A.count(i)
        elif A.count(i)>=B.count(i):
            res *= i**B.count(i)

if  N[0] == N[-1]:
    x = 0
    y = 1
elif N[0]!=N[-1]:
    for i in range(max(N)):
        for j in range(min(N)):
            if min(N) * j - max(N)*i == res:
                x = -i
                y = j
            elif max(N)*i - min(N) * j == res:
                y = -j
                x = i

if N[0]>N[-1]:
    print(x,y, res)
elif N[-1]>=N[0]:
    print(y,x,res)