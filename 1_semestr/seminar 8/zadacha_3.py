import math
road = int(input())
arargrgregeragg=1
while road!=0:
    n, k = map(int, input().split(' '))
    l = [0]*n
    for i in range(n):
        l[i]=list(map(float, input().split()))
    r =  0
    put = []
    for i in range(len(l) - 1):
        r += ((l[i + 1][0] - l[i][0])**2 + (l[i + 1][1] - l[i][1])**2)**0.5
    interval = r / (k - 1)

    x = l[0][0]
    y = l[0][1]
    new = [[x,y]]
    for i in range(n-2):
        shit=1
        alfa = math.atan((l[i+1][1]-l[i][1])/(l[i+1][0]-l[i][0]))
        r = ((x - l[i + 1][0]) ** 2 + (y - l[i + 1][1]) ** 2) ** 0.5
        #print(l[i], l[i+1])
        #print(alfa)
        while interval*shit <= r:
            if (l[i+1][1] - l[i][1])<0 and (l[i+1][0] - l[i ][0]) < 0:
                x -= interval * math.cos(alfa)
                y -= interval * math.sin(alfa)
            else:
                x += interval * math.cos(alfa)
                y += interval * math.sin(alfa)
            new.append([x, y])
            shit+=1
        else:
            peremenaya = interval
            r = ((x - l[i + 1][0]) ** 2 + (y - l[i + 1][1]) ** 2) ** 0.5
            peremenaya -= r
            alfa = math.atan((l[i + 2][1] - l[i+1][1]) / (l[i + 2][0] - l[i+1][0]))
            if (l[i + 2][1] - l[i+1][1]) < 0 and (l[i + 2][0] - l[i+1][0]) < 0:
                x = l[i + 1][0] - peremenaya * math.cos(alfa)
                y = l[i + 1][1] - peremenaya * math.sin(alfa)
            else:
                x = l[i + 1][0] + peremenaya * math.cos(alfa)
                y = l[i + 1][1] + peremenaya * math.sin(alfa)
            new.append([x, y])
            n-=1

    if len(new)>n-2:
        new.pop(-1)
    new.append([l[-1][0],l[-1][1]])

    print('Road #'+str(arargrgregeragg)+':')
    for j in range(k):
        if j == 0 or j == k-1:
            print(str(round(new[j][0],2))+'0',str(round(new[j][1],2))+'0')
        else:
            print(round(new[j][0],2),round(new[j][1],2))
    new = []
    road -=1
    arargrgregeragg+=1
'''
2
5 6
10.00 10.00
20.00 20.00
30.00 10.00
10.00 0.00
9.00 9.00
5 6
10.00 10.00
20.00 20.00
30.00 10.00
10.00 0.00
9.00 9.00
'''