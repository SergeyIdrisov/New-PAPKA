with open("E:\input.txt", 'r') as file:
    f1=file.readlines()

a=''
b=0
v=0
m=0
for i in range(len(f1)):
    a += f1[i]

if a.count('. ')!=0:
    b=a.count('. ')
if a.count('? ')!=0:
    v=a.count('? ')
if a.count('! ')!=0:
    m=a.count('! ')
print (b+v+m+1,a)