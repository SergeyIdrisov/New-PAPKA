amount_and_volum = list(map(int,input().split()))
staty = list(map(int, input().split()))
def determine(staty_1):
    shit = []
    for i in staty_1:
        shit_klon = 0
        for f in staty_1:
            if i<= f:
                shit_klon+=1
        shit.append(shit_klon)
    ret = staty_1
    for i in range(len(staty_1)):
        if ret[i]>=shit[i]:
            ret[i] = 0
    return max(ret)
answer = []
staty_klon = []
for e in range(len(staty)):
    staty_klon.append(staty[e])
if amount_and_volum[-1] == 0:
    print(determine(staty_klon))
else:
    for i in range(amount_and_volum[0]-amount_and_volum[-1]+1):
        for e in range(len(staty)):
            staty_klon.append(staty[e])
        for f in range(amount_and_volum[-1]):
            staty_klon[f+i] += 1
        answer.append(determine(staty_klon))
    print(max(answer))