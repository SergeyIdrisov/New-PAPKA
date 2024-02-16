N = list(map(float, input().split()))
center_mass=0
x = 0
y = 0
z = 0
for i in range(int(len(N))):
    if i%3 ==0:
        x += N[i]
    if (i+2)%3==0:
        y+= N[(i)]
    if (i+1)%3==0:
        z+= N[(i)]
Xc = (x/len(N))*3
Yc = (y/len(N))*3
Zc = (z/len(N))*3
print(Xc,Yc,Zc)