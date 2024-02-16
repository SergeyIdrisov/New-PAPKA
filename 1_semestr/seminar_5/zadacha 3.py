import random
from tkinter import *
import pandas as pd
import tkinter as ttk
films = pd.read_csv('imdb_top_250.csv')
film_genres_list = list(films['Genre'])
nugn=list(films['Title'])

complex_genres = []
for film_genre in film_genres_list:
    genres = film_genre.split(' | ')
    if len(genres) > 1:
        for genre in genres:
            film_genres_list.append(genre)
        complex_genres.append(film_genre)

for genre in complex_genres:
    film_genres_list.remove(genre)

genres_set = set(film_genres_list)
def poisk(*args):
    try:
        if film_genres_list.count(str(in_genres.get()))>=1:
            M=[]
            for i in range(len(film_genres_list)):
                A = str(film_genres_list[i])
                b=A.split(' | ')
                if b.count(in_genres.get()) == 1:
                    if i<len(nugn):
                        M.append(nugn[i])
            ansver.set(M[random.randint(0,len(M)-1)])
        else:
            pass
    except ValueError:
        pass

root = Tk()
root.title('Genres film')
in_genres = StringVar()
in_genres_entry = ttk.Entry(width=30, textvariable=in_genres).grid(column=1, row=1, sticky=(W,E))
ansver = StringVar()
ttk.Label(textvariable=ansver).grid(column=1, row=2, sticky=(W,E))
ttk.Button(text='Найти', command=poisk).grid(column=1, row=3, sticky=(W,E))
for child in root.winfo_children():
    child.grid_configure(padx=5, pady=5)
root.bind("<Return>", poisk)
root.mainloop()
print()