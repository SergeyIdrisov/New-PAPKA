n=int(input())
A=list(map(int,input().split()))
a=0
for i in range(len(A)):
    a+=A[i]
print(a//n)