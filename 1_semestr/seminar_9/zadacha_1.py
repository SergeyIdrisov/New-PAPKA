N = list(map(str, input()))
stack = []
res = []
for i in N:
    if i.isdigit():
        res.append(i)
    elif i in ['+', '-', '/', '*']:
        while stack and stack[-1] in ['/','*']:
            res.append(stack.pop())
        stack.append(i)
    elif i == '(':
        stack.append(i)
    elif i == ')':
        while stack and stack[-1] != '(':
            res.append(stack.pop())
        stack.pop()
while stack:
    res.append(stack.pop())
WHAT_IT_THIS_MAZAFAKAAAA = ''
for i in res:
    WHAT_IT_THIS_MAZAFAKAAAA+=i
print(WHAT_IT_THIS_MAZAFAKAAAA)
stack = []
res = []
for i in N[::-1]:
    if i.isdigit():
        res.append(i)
    elif i in ['+', '-', '/', '*']:
        while stack and stack[-1] in ['/','*']:
            res.append(stack.pop())
        stack.append(i)

    elif i ==')':
        stack.append(i)
    elif i == '(':
        while stack[-1] != ')':
            res.append(stack.pop())
        stack.pop()
for i in range(len(stack)):
    res.append(stack.pop())
WHAT_IT_THIS_MAZAFAKAAAA = ''
for i in res[::-1]:
    WHAT_IT_THIS_MAZAFAKAAAA+=i
print(WHAT_IT_THIS_MAZAFAKAAAA)

#(2-3)*(12-10)+4/2
# + * - 23 - 1210 /42