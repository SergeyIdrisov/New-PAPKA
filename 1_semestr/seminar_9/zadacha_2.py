N = input().split()
stack = []
flag = True
for i in N:
    try:
        if i == '+':
            a2 = int(stack[0])
            a1 = int(stack[1])
            r = a1 + a2
            stack = [r] + stack[2:]
        elif i == '-':
            a2 = int(stack[0])
            a1 = int(stack[1])
            r = a1 - a2
            stack = [r] + stack[2:]
        elif i == '*':
            a2 = int(stack[0])
            a1 = int(stack[1])
            r = a1 * a2
            stack = [r] + stack[2:]
        elif (i == '/'):
            a2 = int(stack[0])
            a1 = int(stack[1])
            r = int(a1) // int(a2)
            stack = [r] + stack[2:]
        elif i.isdigit():
            stack += [i]
    except:
        print("Не не не, меня не проведёшь, мой \'calc\' \nсамый лучший на районе...=)")
        flag = False

if flag:
    print(*stack)
#2 3 - 12 10 - * 4 2 / +