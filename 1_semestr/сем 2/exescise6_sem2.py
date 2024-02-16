st=input()
a=str
b= []
for i in range(len(st)):
    a=st[i]
    if st.count(a)==1:
        b.append(*a)
print(*b)

