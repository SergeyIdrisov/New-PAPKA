f = open("E:\input.txt", 'r')
A = []
for line in f:
    A.append(line)
f.close()

# A[0] -- строка чисел, разделенных пробелами (str)
# A[1] -- знак арифм операции (str)
# A[2] -- основание системы сч. (str)

nums = list(map(int, A[0].split()))
c = int(A[2])
nums_dec = []
for i in range(len(nums)):  # идем по числам
    num_str = str(nums[i])
    num_dec = 0
    for j in range(len(num_str)):  # каждое число переводим в десятичную
        num_dec += int(num_str[-(j + 1)]) * c ** j
    nums_dec.append(num_dec)  # записываем

# Шаг 2: просуммировать/перемножить числа в списке nums_dec
# Шаг 3: перевезаписать  результат в c-ичную систему счисления и вывести в output
ver = 1
for i in range(len(nums_dec)):
    if str(A[1]) == '+\n' or str(A[1]) == '+':
        ver += nums_dec[i]
    elif str(A[1]) == '*\n' or str(A[1]) == '*':
        ver *= nums_dec[i]
    elif str(A[1]) == '-' or str(A[1]) == '-\n':
        ver = ver - nums_dec[i]

if str(A[1]) == '+\n' or str(A[1]) == '+' or str(A[1]) == '-' or str(A[1]) == '-\n':
    ost = ver - 1
    ans = []
    while ost != 0:
        ans.append(ost%c)
        ost//=c

elif str(A[1]) == '*\n' or str(A[1]) == '*':
    ost = ver
    ans = []
    while ost != 0:
        ans.append(ost % c)
        ost //= c


ans = list(map(str, ans))
f=open('E:\output.txt','w')
f.write(''.join(ans[::-1]))
f.close()

