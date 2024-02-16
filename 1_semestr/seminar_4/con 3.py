A=list(map(int, input().split()))
res=[]
i=0
while len(res) != 1:
    if (A[1]+i)%A[0]==0 and (A[0]+i)%A[1] == 0:
        res.append(i)
    i+=1
print(res[0])