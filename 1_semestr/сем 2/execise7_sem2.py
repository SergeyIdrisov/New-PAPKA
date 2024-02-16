st=input()
a=str
b= []
g=0
for i in range(len(st)):
    a=st[i]
    if st.count(a)>g:
        b.append(*a)
        g+=1
print(*b[-1])