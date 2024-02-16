Volum = list(map(int, input().split()))
Dp = [0]*len(Volum)
for i in range(len(Volum)):
    Dp[i] = max(Volum[i]+Dp[i-1], Volum[i]+Dp[i-2])
print(Dp[-1])