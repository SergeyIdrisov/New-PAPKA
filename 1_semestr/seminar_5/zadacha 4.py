from tkinter import *
root = Tk()
root.title('Hex')

frame = Frame(root, padx=10, pady=10)
frame.pack(expand=True)

text = Label(frame, text='Hex')
text.grid(row=1, column=1)
enter = Entry(frame)
enter.grid(row=2, column=1)

def calculator():
    gt = enter.get()
    a = int(gt[1:3], 16)
    b = int(gt[3:5], 16)
    c = int(gt[5:], 16)
    enter.delete(0, last=END)
    enter.insert(0, '#' + hex(255 - a)[2:] + hex(255 - b)[2:] + hex(255 - c)[2:])

btn = Button(frame, text='Подсчёт...', command=calculator)
btn.grid(row=3, column=1)

root.mainloop()