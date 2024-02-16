from tkinter import *
from tkinter import ttk
from random import randint
import random

WIDTH = 300
HEIGHT = 200


class Ball:
    def __init__(self):
        color = ['blue','red','green','yellow','white','pink','black','gold','grey','purple','orange']
        self.R = randint(10, 50) #храним размер, при каждом создании объекта будет выбираться случайно
        self.x = randint(self.R, WIDTH - self.R) # храним положение по x и y
        self.y = randint(self.R, HEIGHT - self.R)
        self.dx, self.dy = (5, 5) # это по сути шаг движения шаров. если увеличить -- будут двигаться быстрее
        self.ball_id = canvas.create_oval(self.x - self.R,
                                     self.y - self.R,
                                     self.x + self.R,
                                     self.y + self.R, fill=random.choice(color)) # при создании шарика отрисовываем его

    def move(self):
        self.x += self.dx
        self.y += self.dy
        if self.x + self.R > WIDTH or self.x - self.R <= 0: # отражение от стенок
            self.dx = -self.dx
        if self.y + self.R > HEIGHT or self.y - self.R <= 0: # отр
            self.dy = -self.dy

    def show(self):
        canvas.move(self.ball_id, self.dx, self.dy)


def click_handler(event):
    for ball in balls:
        if (event.x-ball.x)**2+(event.y-ball.y)**2<=ball.R**2:
            balls.append(Ball())
            break

#здесь мы уже привычно обращаемся к balls как к глобальной переменной. На самом деле дело в том, что нам лень писать классы.
def tick():
    for ball in balls:
        ball.move()
        ball.show()
    root.after(20, tick)


root = Tk()
root.geometry(f'{WIDTH}x{HEIGHT}')
canvas = Canvas(root)
canvas.pack()
#сделаем так, чтобы нажатие левой кнопки на поле выводило координаты точки, в которую мы нажали
balls = [Ball() for i in range(5)]

canvas.bind('<Button-1>', click_handler)
# делаем шаг перемещения и отрисовки шаров. поскольку mainloop циклит наше приложение, это будет происходить, пока мы не закроем окно
tick()
root.mainloop()
