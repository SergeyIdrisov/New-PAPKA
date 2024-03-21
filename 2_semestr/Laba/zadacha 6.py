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
        print(sigma2, sigma1)

        x=[]
        y=[]
        x.append(MNK1(x1,y1))
        x.append(MNK2(x1,y1))
        sum1, sum2, = 0,0
        for i in range(len(x1)):
            sum1 +=x1[i]
            sum2 +=y1[i]
        sred1 = sum1 / len(x1)
        sred2 = sum2 / len(x1)

        Koeff = [a_b(x1,y1)[-1]]
        print(Koeff)
        print('Коэффицент наклона k,свободный член b и средняя ошибка  в % соответсвенно равны = ' + str(a_b(x1,y1)))
        ax1.errorbar (x1,y1, yerr = sred2*Koeff[0]/100, xerr = sred1*Koeff[0]/100, color = 'k', linestyle = 'None', marker = 'o')
        plt.xlabel(naz1.get())
        plt.ylabel(naz2.get())
        plt.title(titl.get())
        for i in range(len(x1)):
            b =x[-1] + x[0]*x1[i]
            y.append(b)
        if line.get() == 'p':
            ax1.plot(x1,y,'--r', label = 'Зависмость '+ naz2.get() + ' от '+ naz1.get())

        else:
            ax1.plot(x1, y, 'r',label = 'Зависмость '+ naz2.get() + ' от '+ naz1.get())

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
#1.608 1.5678 1.53 1.496 1.4625 1.438 1.4215 1.4095 1.4145 1.43 1.4645
#687.59956 626.02348 564.4474 502.87131999999997 441.29524 379.71916 318.14307999999994 256.567 194.99091999999996 133.41484 71.83876

#9.8 19.6 29.4 39.2 49 58.8 68.6 78.4 88.2 98 107.8 117.6 127.4 137.2 147 156.8 176.4 196 215.6 235.2 254.8 274.4 297.92 313.6 335.16 350.84 372.4 401.8
#5.78333E-06 1.19167E-05 1.77167E-05 2.44667E-05 3.19333E-05 0.00003675 4.28667E-05 0.00004995 5.54167E-05 6.29667E-05 6.64333E-05 0.00007305 7.98667E-05 8.24167E-05 9.00833E-05 9.22833E-05 9.41833E-05 0.00009645 0.0001003 0.000104583 0.000109367 0.000111033 0.000115167 0.000118283 0.000121117 0.00012265 0.000126383 0.000132617

#58.8 68.6 78.4 99.96 117.6 137.2 156.8 176.4 196 225.4 254.8 284.2 311.64 333.2 352.8
#0.000111817 0.000123117 0.0001324 0.000144817 0.0001511 0.0001611 0.0001724 0.000183833 0.000194717 0.000211317 0.000224867 0.0002386 0.000250867 0.000259933 0.000268283

#  49 158.76 88.2 70.56 88.2
#  0 11.5 11.5 41.5 81.5

#0.000111817 0.000123117 0.0001324 0.000144817 0.0001511 0.0001611 0.0001724 0.000183833 0.000194717 0.000211317 0.000224867 0.0002386 0.000250867 0.000259933 0.000268283

#7.668115805 8.282511696 8.854377448 9.9979998 10.84435337 11.71324037 12.52198067 13.28156617 14 15.01332741 15.96245595 16.85823241 17.6533283 18.25376673 18.78297101

