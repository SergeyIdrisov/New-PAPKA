slov = {}
def fib(n):
    if n in slov:
        return slov[n]
    if n == 4:
        reselt = 4
    elif n<4:
        reselt = 1
    else:
        reselt = (-fib(n-1)+fib(n-2))*fib(n-2)
    slov[n] = reselt
    return reselt
shit = 1
N=[]
while shit==1:
    N.append(int(input()))

    if N[-1] == 0:
        N.pop(-1)
        shit=0
for i in range(len(N)):
    if N[i] > 4:
        print(fib(N[i]))
    elif N[i]==3:
        print(1)
    elif N[i]==4:
        print(3)
    else:
        print(0)
