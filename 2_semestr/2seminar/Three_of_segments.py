import numpy as np


class SumTree:

    def __init__(self, data: list):
        ln = len(data)
        lb = np.log2(ln)
        if lb == int(lb):
            self.data = data
            self.help1 = self.data
        else:
            self.data = data
            self.help1 = self.data
            lb = int(lb) + 1
            for i in range(ln, 2 ** lb):
                self.data.append(0)

        self.tree = [0 for i in range(len(self.data) - 1)] + self.data
        self.calc_tree()


    def calc_tree(self) -> None:
        for i in range(len(self.tree) + 1, 2, -2):
            s1 = self.tree[i - 2]
            s2 = self.tree[i - 3]
            sm = s1 + s2
            self.tree[(i - 4) // 2] = sm

def update(self, l: int, r: int, number: int, i=0):
        r+=1
        while r !=l:
            self.help1[l + i] += number
            i+=1
            r-=1
        self.tree = self.help1


def Sum(self, l: int, r: int):

    def tree_sum(l: int, r: int, tl=0, tr=len(self.data) - 1):  # len(tree.data)-1


        #root = tree.tree[0]
        # left from root - [0,(len(self.data))//2]
        # right from root - [len(self.data)//2,len(self.data)]
        sum = 0

        # если встретили лист
        tm = (tl + tr + 1) // 2
        if tr == tl:
            return self.data[tm]

        # если отрезок на который смотрим полностью внутри запрашиваемого
        if l <= tl and r >= tr:

            index1 = (len(self.data) - 1) + tr  # 14
            index2 = (len(self.data) - 1) + tl + 1  # 8

            while index1 != index2:
                index1 = max((index1 - 2) // 2, 0)  # 6
                index2 = max((index2 - 2) // 2, 0)  # 3

            if tl + 1 == tr:
                return self.tree[(index1 - 2) // 2]
            else:
                return self.tree[index1]

        # если надо, спускаемся по дереву дальше
        go_left = l < tm
        go_right = r >= tm

        #print(go_left, go_right, tl, tr, tm)

        if go_left:
            sum += tree_sum(l, r, tl=tl, tr=tm - 1)
        if go_right:
            sum += tree_sum(l, r, tl=tm, tr=tr)
        return sum
    if l == 0:
        l = 1
        return (tree_sum(l,r)+self.help1[0])
    else:
        return (tree_sum(l,r))

# [1,2,3,4,5,6,7,8]
# 0 1 2 3 4 5 6
tree = [1,2,3,4,5,6,7,8]

print(Sum(SumTree(tree),0,5))

update(SumTree(tree),0, 5, 5)


print(Sum(SumTree(tree), 0,5))


