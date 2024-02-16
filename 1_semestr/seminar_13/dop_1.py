from math import *

#N=int(input())

def main(N):
    B= N
    i=2
    res=[]
    if str(B).count('.'):
        raise ValueError
    if type(B) not in [int]:
        raise TypeError
    if B==1:
        res.append(1)
        return res
    while i<=int(sqrt(B)):
        if N%i == 0:
            N = N // i
            res.append(i)
            i-=1
        i+=1
    if N==1:
        return (res)
    else:
        res.append(N)
        return (res)
#print(*main(N))