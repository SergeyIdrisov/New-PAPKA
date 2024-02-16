n = int(input())
b,a=0,0
for i in range(n-1):
    b+=int(input())
for i in range (n+1):
    a+=i
print (a-b)
