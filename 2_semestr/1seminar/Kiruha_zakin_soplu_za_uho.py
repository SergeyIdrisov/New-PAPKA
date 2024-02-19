import seaborn as sns
import pandas as pd
import matplotlib.pyplot as plt
import argparse



parser = argparse.ArgumentParser()
parser.add_argument("month", type=str)
parser.add_argument("year", type=str)
args = parser.parse_args()
try:
    tab = pd.read_excel(f'/home/kirill/outcome_{args.month}.{args.year}.xlsx')


    tab["День"]=[int(x.split()[0]) for x in tab['Дата']]


    fig, ax1, ax2 = plt.subplots(constrained_layout = True)

    sns.lineplot(data=tab, x="День", y="Сумма", hue="Категория", ax=ax1)
    ax1.set_title(f'{args.month}.{args.year}')
    ax1.legend()



    sns.barplot(data=tab, x="Сумма", y="Категория", orient='h', estimator='sum', errorbar=None, ax=ax2)
    ax2.set_title(f'{args.month}.{args.year}')
    ax2.legend()
    plt.savefig('1.png')
except FileNotFoundError:
    print("Нет необходимого документа")
# In[ ]: