x =[]
y =[]
Flag = True
for i in range(int(input())):
    N = list(map(int, input().split()))
    x.append(N[0])
    y.append(N[1])
b = (x[0]+x[-1])/2
for i in range(len(x)):
    x.append(x[0]-b)
    x.pop(0)
for i in range(len(x)):
    if -x[i]!=x[-i-1]:
        Flag = False
        break
if Flag:
    print('Yes')
else:
    print('No')