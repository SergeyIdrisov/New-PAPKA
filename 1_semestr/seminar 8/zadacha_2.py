b = 1
N = []
while b != 0:
    b = int(input())
    N.append(b)  # 5
    if b == 0:  # 8
        shit = 0  # 0
        N.pop(-1)
res = [0, 1, 3]
for i in range(len(N)):
    if N[i] <= 2:
        print(0)
    elif N[i] == 3:
        print(1)
    else:
        for j in range(5, N[i] + 1):
            res.append(2**(j - 3) + res[-1] + res[-2] + res[-3])
        print(res[N[i] -2])
        res = [0, 1, 3]
'''lst = [list(i) for i in itertools.product([0, 1], repeat=3)]
for i in lst:
    print(i)'''