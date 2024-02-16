N = int(input())
def fib (N):
    dp = [0]
    if N == 1:
        return 1
    if N == 0:
        return 0
    if N == 2:
        return 2
    if N > 2:
        dp[-1] = fib(N-1) + fib(N-2)
    return dp[-1]
print(fib(N))