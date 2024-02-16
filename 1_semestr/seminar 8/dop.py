expression = "2 * (3 + 4)"
stack = []
output = []

for token in expression:
    if token.isdigit():
        output.append(token)
    elif token in ["+", "-", "*", "/"]:
        while stack and stack[-1] in ["*", "/"]: # проверяем приоритет оператора
            output.append(stack.pop())
        stack.append(token)
    elif token == "(":
        stack.append(token)
    elif token == ")":
        while stack and stack[-1] != "(":
            output.append(stack.pop())
        stack.pop()

while stack:
    output.append(stack.pop())

print(' '.join(output))