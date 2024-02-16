import numpy as np
class SumThree:
    def __init__(self, data: list):
        ln = len(data)
        lb = np.log2(ln)
        if lb == int(lb):
            self.data = data
        else:
            self.data = data
            lb = int(lb) + 1
            for i in range(ln, 2**lb):
                self.data.append(0)

        self.tree = [0 for i in range(len(self.data)-1)] + self.data
        self.calc_tree()
    def calc_tree(self)-> None:
        for i in range(len(self.tree)+1,2, -2):
            s1 = self.tree[i-2]
            s2 = self.tree[i-3]
            sm = s1+s2
            self.tree[(i-4)//2] = sm
def Sum(self, l:int, r:int):
    def tree_sum( l:int, r:int, tl = 0, tr = len(self.data)-1):

        root = self.tree[0]
            #left from root - [0,len((self.data)//2)]
            #right from root - [len(self.data)//2, len(self.data)]

        if tl==tr:
            return self.data[tl]


        if l<= tl and r>=tr:
            index1 = len(self.data) +tr-1
            index2 = len(self.data) + tl
            while index2!=index1:
                index1 = max((index1-2)//2, 0)
                index2 = max((index2-2)//2, 0)
            return self.tree[index1]

        go_left = l < (len(self.data))//2
        go_right = r >= (len(self.data)) // 2
        tm = (tl+tr)//2
        if go_left:
            tree_sum( l, r, tl = tl, tr = tm-1)
        if go_right:
            tree_sum( l, r, tl = tm, tr = tr)
    return tree_sum(l,r)

def repr (trfdf):
    if str(trfdf) == 'None':
        print('Pipiska Dinozavra TEBYA NYAM NYAM')
    else:
        print(trfdf)








thre = SumThree([1,1 ,2, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0])
print(thre.data)
print(thre.tree)
repr(Sum(thre,5,7))


