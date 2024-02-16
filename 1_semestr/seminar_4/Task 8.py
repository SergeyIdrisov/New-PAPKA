A = list(map(int, input().split()))
B = list(map(int, input().split()))
ind_A=[]
ind_B=[]
ind_U=[]
Sov=[]
for i in range(len(B)):
    if B.count(B[i]) == 1:
        ind_B.append(B[i])
for i in range(len(A)):
    if A.count(A[i])==1:
        ind_A.append(A[i])
U = []
for i in range(len(A)):
    U.append(A[i])
for i in range(len(B)):
    U.append(B[i])
for i in range(len(U)):
    if U.count(U[i]) == 1:
        ind_U.append(U[i])
for i in range(max(U)):
    if i in U == True:
        Sov.append(i)
print(ind_A,ind_B,ind_U,Sov,max(U))