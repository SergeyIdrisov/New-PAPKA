
n = list(map(int, input().split()))
def huytirovka(n):
    lox = len(n)
    i = lox // 2 - 1
    while i != -1:
        if i * 2 + 1 >= lox:
            i -= 1
            continue
        elif i * 2 + 1 == lox - 1:
            if n[i] < n[2 * i + 1]:
                n[i], n[2 * i + 1] = n[2 * i + 1], n[i]

        elif 2 * i + 2 <= lox - 1:
            if n[i] < n[2 * i + 1] or n[i] < n[2 * i + 2]:
                if n[2 * i + 1] > n[i * 2 + 2]:
                    n[i], n[2 * i + 1] = n[2 * i + 1], n[i]
                elif n[2 * i + 1] < n[i * 2 + 2]:
                    n[i], n[2 * i + 2] = n[2 * i + 2], n[i]
        i -= 1

        if i <= -1:
            break

    return n

for i in range(5):
    n = huytirovka(n)
print(*n)