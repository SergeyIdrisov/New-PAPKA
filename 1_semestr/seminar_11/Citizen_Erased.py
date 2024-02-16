amount_section = int(input())
amount_section_klon = amount_section
dot_section = []
while amount_section_klon != 0:
    dot_section.append(list(map(int,input().split())))
    amount_section_klon -=1
Flag = True
while Flag:
    w=0
    for i in range(amount_section-1):
        if dot_section[i][-1] > dot_section[i+1][-1]:
            dot_section[i],dot_section[i+1] = dot_section[i+1],dot_section[i]
        else:
            w+=1
        if w == amount_section-1:
            Flag = False
answer = []
answer.append(dot_section[0])
for i in range(amount_section):
    if answer[-1][-1]<dot_section[i][0]:
        answer.append(dot_section[i])
print(len(answer))
