import os
import sys
import numpy as np
import matplotlib.pyplot as plt

#filename = 'graphene_all_3351000..log'

#os.chdir('./Sem1_dimers_data/')
filenames = os.listdir('./Sem1_dimers_data/')[1:] #'./'
output_name = sys.argv[1]
os.remove(f'{output_name}')
for filename in filenames:
    os.system(f'python 1_sem.py {filename} {output_name}')
with open(f'{output_name}','r') as file:
    data = file.readlines()
res_data=[]
for num in data:
    if num != 'None\n':
        res_data.append(float(num))

plt.hist(res_data, bins = [-9000, -8950, -8900, -8850, -4600, -4475,-4450])

plt.show()
#print(filenames)
