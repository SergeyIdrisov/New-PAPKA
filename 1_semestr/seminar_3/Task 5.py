N = int(input())
M = int(input())
i, j = 0, 0
a=''
b=0
while b<M*N:
    b+=1
    while i<N:
        i += 1
        a+=str(i)

print(b,i,a)