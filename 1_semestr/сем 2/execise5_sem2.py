A = list(map(int,input().split()))
for i in range(len(A)-1):
    A[-i],A[-(1+i)]=A[-(1+i)],A[-i]
print(A)