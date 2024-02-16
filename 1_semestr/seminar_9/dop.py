results = []
while True:
    N, D = map(int, input().split())
    if N == 0 and D == 0:
        break

    num = input().strip()
    # print(num)
    # print(type(num))

    stack = []
    remaining_digits = N - D
    # print(type(remaining_digits))

    for digit in num:
        # print(type(digit))
        while stack and D > 0 and stack[-1] < digit:
            stack.pop()
            D -= 1
        # print(digit)
        stack.append(digit)

    result = ''.join(stack[:remaining_digits])
    results.append(result)

for item in results:
    print(item)