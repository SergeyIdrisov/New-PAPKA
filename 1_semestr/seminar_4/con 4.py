A=list(map(int, input().split()))
def f(n1):
    i=0
    while n1 !=0:
        if n1 % 2 == 0:
            return i
        n1-=1
        n1/=2
        i+=1
    return i
B=0
if f(A[0])>=f(A[1]):
    for i in range(f(A[1])):
        B+=4**i
else:
    for i in range(f(A[0])):
        B+=4**i

print(B)