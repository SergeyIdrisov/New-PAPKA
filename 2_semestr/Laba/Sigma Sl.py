from tkinter import *
import matplotlib.pyplot as plt
from math import *

root = Tk()
root.title("Лаба")

root.grid_rowconfigure(index=0, weight=1)
root.grid_columnconfigure(index=0, weight=1)
root.grid_columnconfigure(index=1, weight=1)

shit_1 = StringVar()
shit_2 = StringVar()
line=StringVar()
titl = StringVar()
naz1 = StringVar()
naz2 = StringVar()
shit_1_entry=Entry(width=30, textvariable=shit_1).grid( column=0, row=0, sticky=EW, padx=10)
shit_2_entry = Entry(width=30,textvariable=shit_2).grid(column=1, row=0, sticky=EW, padx=10)
line_entry=Entry(width=30,textvariable=line).grid(column=2, row=0, sticky=EW, padx=10)
titl_entry = Entry(width=30,textvariable=titl).grid(column=0, row=2, sticky=EW, padx=10)
naz1_entry = Entry(width=30,textvariable=naz1).grid(column=1, row=2, sticky=EW, padx=10)
naz2_entry = Entry(width=30,textvariable=naz2).grid(column=2, row=2, sticky=EW, padx=10)
Label(text='Введите название графика.').grid(column=0, row=3, sticky=EW, padx=10)
Label(text='Введите значения для оси ОХ').grid(column=0, row=1, sticky=EW, padx=10)
Label(text='Введите значения для оси OY').grid(column=1, row=1, sticky=EW, padx=10)
Label(text='Введите "p" - для пунктирной линии.').grid(column=2, row=1, sticky=EW, padx=10)
Label(text='Введите название оси ОХ.').grid(column=1, row=3, sticky=EW, padx=10)
Label(text='Введите название оси ОY.').grid(column=2, row=3, sticky=EW, padx=10)

for child in root.winfo_children():
    child.grid_configure(padx=5, pady=5)

def graphick1():
    try:
        fib = plt.figure(figsize=(16,9))
        ax1 = fib.add_subplot()
        x1 = list(map(float, shit_1.get().split()))
        y1 = list(map(float, shit_2.get().split()))


        def MNK1(x,y):
            xy = 0
            x2 = 0
            y_ = 0
            x_ = 0
            for i in range(len(x)):
                xy += x[i] * y[i]
                x_ += x[i]
                y_ += y[i]
                x2 += x[i] * x[i]
            xy /= len(x)
            x_ /= len(x)
            y_ /= len(x)
            x2 /= len(x)
            b = (xy - x_ * y_) / (x2 - x_ ** 2)
            a = y_ - b * x_
            return b
        def MNK2(x,y):
            xy = 0
            x2 = 0
            y_ = 0
            x_ = 0
            for i in range(len(x)):
                xy += x[i] * y[i]
                x_ += x[i]
                y_ += y[i]
                x2 += x[i] * x[i]
            xy /= len(x)
            x_ /= len(x)
            y_ /= len(x)
            x2 /= len(x)
            b = (xy - x_ * y_) / (x2 - x_ ** 2)
            a = y_ - b * x_
            return a
        def a_b(x,y):
            Ex=0
            Ey=0
            Exy = 0
            Ex2 = 0
            sum = len(x)
            A = 0
            for i in range(sum):
                Ex += x[i]
                Ey += y[i]
                Exy += x[i]*y[i]
                Ex2 += x[i]**2
            a = (Ex*Ey - sum*Exy)/(Ex**2-sum*Ex2)
            b = (Ex*Exy-Ex2*Ey)/(Ex**2-sum*Ex2)
            for i in range(sum):
                y_sr = a*x[i] + b
                A += abs((y[i]-y_sr)/y[i])
            e = (A/sum)*100
            return a,b,e

        sred = 0
        for i in range(len(x1)):
            sred += x1[i]
        sred /= len(x1)
        sum = 0
        for i in range(len(x1)):
            sum += (sred - x1[i]) ** 2
        sigma1 = sqrt(sum / (len(x1) - 1))

        sred = 0
        for i in range(len(y1)):
            sred += y1[i]
        sred /= len(y1)
        sum = 0
        for i in range(len(y1)):
            sum += (sred - y1[i]) ** 2
        sigma2 = sqrt(sum / (len(y1) - 1))

        x=[]
        y=[]
        x.append(MNK1(x1,y1))
        x.append(MNK2(x1,y1))
        ax1.errorbar (x1,y1, yerr = sigma2/100, xerr = sigma1/100, color = 'k', linestyle = 'None', marker = 'o')
        plt.xlabel(naz1.get())
        plt.ylabel(naz2.get())
        plt.title(titl.get())
        for i in range(len(x1)):
            b =x[-1] + x[0]*x1[i]
            y.append(b)
        if line.get() == 'p':
            ax1.plot(x1,y,'--r', label = 'Зависмость '+ naz1.get() + ' от '+ naz2.get())

        else:
            ax1.plot(x1, y, 'r',label = 'Зависмость '+ naz1.get() + ' от '+ naz2.get())
        print('Коэффицент наклона k,свободный член b и средняя ошибка соответсвенно равны = ' + str(a_b(x1,y1)))
        plt.legend()
        plt.show()
    except AttributeError:
        pass
def graphick2():
    try:
        fib = plt.figure(figsize=(16, 9))
        ax1 = fib.add_subplot()
        x1 = list(map(float, shit_1.get().split()))
        y1 = list(map(float, shit_2.get().split()))
        ax1.errorbar(x1, y1, color='k', linestyle='None', marker = 'o')
        plt.title(titl.get())
        plt.xlabel(naz1.get())
        plt.ylabel(naz2.get())
        if line.get() == 'p':
            ax1.plot(x1, y1, '--r',label = 'Зависмость '+ naz1.get() + ' от '+ naz2.get())
        else:
            ax1.plot(x1, y1, 'r',label = 'Зависмость '+ naz1.get() + ' от '+ naz2.get())
        plt.legend()
        plt.show()
    except AttributeError:
        pass
Button(text='Линейная апроксимация', command=graphick1).grid( column = 0, row=4, sticky=EW, padx=10)
Button(text='Соединить точки', command=graphick2).grid( column = 1,row=4,sticky=EW,padx =10)
root.mainloop()
#9.85 9.83 9.81 9.80 9.83 9.83 9.83 9.89 9.89 9.95 10.00
