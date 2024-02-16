A = list(map(int, input().split()))
B = [0]*len(A)
i=0
while i < len(A):
    if i+3<len(A):
        if A[i] + A[i+2]>= A[i] + A[i+3] >= A[i+1]+A[i+3]:
            B[i]=A[i]+A[i+2]
            i+=2
        elif A[i] + A[i+2]<= A[i] + A[i+3] <= A[i+1]+A[i+3]:
            B[i]=A[i+1]+A[i+3]
            i+=3
        else:
            B[i]=A[i] + A[i+3]
            i+=3
    else:
        B[i]=max(A[-3:-1])
        i+=3
res=0
for i in range(len(B)):
    res+=B[i]
print(res,B)
