
n = int(input())
x = []
y = []
for i in range(n):
    x.append(float(input()))
for i in range(n):
    y.append(float(input()))
sum_xy = 0
sum_x2 = 0
sum_y2 = 0
for i in range(n):
    sum_xy += x[i]*y[i]
for i in range(n):
    sum_x2 += x[i]**2
for i in range(n):
    sum_y2 += y[i]**2
sum_xy = sum_xy/n
sum_x2 = sum_x2/n
sum_y2 = sum_y2/n
k = sum_xy/sum_x2
sigma_k = (1/(n-1))*((sum_y2/sum_x2) - k**2)**(1/2)
print(k)
print(sigma_k)