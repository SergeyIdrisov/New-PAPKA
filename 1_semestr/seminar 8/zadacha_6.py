shit = 1
while shit != 0:
    N = input().split()
    a = int(N[0])
    b = int(N[-1])
    if a == 0:
        break
    numbers = input()
    if a == b:
        print(numbers)

    stack = []
    shit = a-b
    for i in numbers:
        while stack and b>0 and stack[-1] < i:
            stack.pop()
            b-=1
        stack.append(i)
    ans = ''
    for i in range(shit):
        ans += stack[i]
    print(ans)

"""results = []
while True:
    N, D = map(int, input().split())
    if N == 0 and D == 0:
        break

    numbers = input().strip()

    stack = []
    shit = N - D

    for digit in numbers:
        while stack and D > 0 and stack[-1] < digit:
            stack.pop()
            D -= 1
        print(digit)
        stack.append(digit)

    result = ''.join(stack[:remaining_digits])
    results.append(result)

for item in results:
    print(item)"""
'''32 16
37593759375937593759375937593759
13 7
1210000003123
11 7
10312300000
0 0

9999975937593759
213123
3300'''