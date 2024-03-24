
def g(N, start):
    INF = 10 ** 9
    F = [[INF] * N for i in range(N)]
    F[0][start] = 0
    for k in range(1, N):
        for i in range(N):
             F[k][i] = F[k - 1][i]
             for j in range(N):
                 if F[k - 1][j] + W[j][i] < F[k][i]:
                     F[k][i] = F[k - 1][j] + W[j][i]