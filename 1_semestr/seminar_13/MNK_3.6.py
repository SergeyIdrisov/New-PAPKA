x = list(map(float,input().split()))
y = list(map(float, input().split()))
xy = 0
x2 = 0
y_=0
x_=0
for i in range(len(x)):
    xy += x[i]*y[i]
    x_ += x[i]
    y_ += y[i]
    x2 += x[i]*x[i]
xy /= len(x)
x_ /= len(x)
y_ /= len(x)
x2 /= len(x)
b = (xy-x_*y_)/(x2 - x_**2)
a = y_-b*x_
print (a,b)