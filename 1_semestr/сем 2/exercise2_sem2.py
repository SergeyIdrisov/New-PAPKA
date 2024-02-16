s = input().split()
G = int(s[0])
st = s[1]

L=len(st)
num = L//G
A=[]
for i in range (num):
    A.append(st[i*G:(i+1)*G])
B=[]
for i in range (len(A)):
    word = A[i]
    word_1=word[::-1]
    B.append(word_1)
res = ''.join(B)
print(res)
