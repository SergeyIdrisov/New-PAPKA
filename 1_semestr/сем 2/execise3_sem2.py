st = input()
L = len(st)
num=L//2
flag = True
for i in range (num):

    if st[i] == 'E' and st[-(1+i)] !='3':
        flag = False
        break
    elif st[i] == 'Z' and st[-(1+i)] !='5':
        flag = False
        break
    elif st[i] == 'S' and st[-(1+i)] !='2':
        flag = False
        break
    elif st[i] == 'J' and st[-(1+i)] !='L':
        flag = False
        break
    elif st[i] != 'S' or st[i] != 'S' or st[i] != '3' or st[i] != 'Z' or st[i] != 'L' or st[i] != 'J' or st[i] != '5' or st[i] != '2':
        if st[i]!=st[-(i+1)]:
            flag = False
            break

print(flag)

res = f'{st}, palindrome = {flag}'
print (res)