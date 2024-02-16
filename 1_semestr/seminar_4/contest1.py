A = list(map(int, input().split()))
B = []
if len(A) > 2:
    B.append(A[0])
    B.append(A[1])
    for i in range(2, len(A)):
        B.append(max(B[i-1], A[i] + B[i-2]))
    res = B[-1]
elif len(A) == 2:
    res = max(A[0], A[1])
else:
    res = A[0]
print(res)