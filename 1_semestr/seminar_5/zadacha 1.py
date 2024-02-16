from tkinter import *
from tkinter import ttk

def calculate(*args):
    try:
        meters.set(eval(feet.get()))
    except ValueError:
        pass

root = Tk()
root.title("Калькулятор")


mainframe = ttk.Frame(root, padding="3 3 12 12")
mainframe.grid(column=0, row=0, sticky=(N, W, E, S))
root.columnconfigure(0, weight=1)
root.rowconfigure(0, weight=1)

feet = StringVar()
feet_entry = ttk.Entry(mainframe, width=15, textvariable=feet)
feet_entry.grid(column=1, row=1, sticky=(W,E))

meters = StringVar()
ttk.Label(mainframe, textvariable=meters).grid(column=1, row=2, sticky=(W))

ttk.Button(mainframe, text="Calculate", command=calculate).grid(column=3, row=3, sticky=W)

ttk.Label(mainframe, text="Выражение").grid(column=3, row=1, sticky=W)
ttk.Label(mainframe, text="Ответ").grid(column=3, row=2, sticky=W)

for child in mainframe.winfo_children():
    child.grid_configure(padx=5, pady=5)

feet_entry.focus()
root.bind("<Return>", calculate)

root.mainloop()