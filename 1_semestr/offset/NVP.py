Number = list(map(int, input().split()))
size = len(Number)
dp = [1]*size
for i in range(size):
    for j in range(i+1):
        if Number[i]>Number[j] and dp[i]<=dp[j]:
            dp [i]+=1
print(max(dp))
#5 10 6 12 3 24 7 8
#5 78 5 4 46 45 46  46 841 4 1 21 54 51 21 2 15 48 78 75 4 232 12 31 4 897 748 51 12 1 4 974 651 2 165 498 7 5 513 1 54894 84 5 15 1 51 54 875 5 1 21 1 44 5 43 186487897 7 8 78 4 54 51 32 12 1  84