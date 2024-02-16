from tkinter import *
from tkinter import ttk
root = Tk()
root.title("Калькулятор ИМТ")
root.geometry('300x150')
def shit(*args):
    try:
        k = int(kg.get())
        w = int(wh.get())/100
        imt.set(int(k/(w**2)))
    except ValueError:
        pass
    if int(imt.get()) < 16:
        table.set('Выраженный дефицит массы тела.')
    elif int(imt.get())>=16 and int(imt.get()) <18.5:
        table.set('Недостаточная (дефицит) масса тела')
    elif int(imt.get())>=18.5 and int(imt.get())<25:
        table.set('Норма')
    elif int(imt.get())>=25 and int(imt.get()) <30:
        table.set('Избыточная масса тела (предожирение)')
    elif int(imt.get()) >= 30 and int(imt.get()) < 35:
        table.set('Ожирение 1 степени')
    elif int(imt.get()) >= 35 and int(imt.get()) < 40:
        table.set('Ожирение 2 степени')
    else:
        table.set('Ожирение 3 степени')
kg = StringVar()
kg_entry = ttk.Entry(width= 7,textvariable = kg)
kg_entry.grid(column = 1,row =1, sticky=(W))
wh = StringVar()
wh_entry = ttk.Entry(width= 7,textvariable = wh)
wh_entry.grid(column = 1,row =2, sticky=(W))
imt = StringVar()
ttk.Label(textvariable=imt).grid(column = 1, row = 3, sticky = (S))
ttk.Button(text=('Посчитать'), command=shit).grid(column = 3,row =3, sticky=(E))
ttk.Label(text='Масса(кг)').grid(column = 2, row = 1, sticky = (W))
ttk.Label(text='Рост(см)').grid(column = 2, row = 2, sticky = (W))
ttk.Label(text='Ваш ИМТ').grid(column = 2, row = 3, sticky = (W))
table = StringVar()
ttk.Label(textvariable=table).grid(column = 1, row = 4, sticky = (W,E))
for child in root.winfo_children():
    child.grid_configure(padx=5, pady=5)
kg_entry.focus()
root.bind("<Return>", shit)
root.mainloop()
