
def init():
        znak = []
        condition = []
        q = int(input())
        for i in range(q):
            condition += [list(map(int, input().split()))]
            znak += list(input().split())
        return  condition, znak
def check(condition):
    znak = condition[1]
    for i in range(len(znak)):
        if znak[i] :
            return True
    condition = condition[0]

def sort(condition):
        condition = condition[0]
        q = len(condition)
        condition_sort = [[1, 100000000000, 0] for _ in range(q)]
        for i in range(q):
            for j in range(q):
                if condition_sort[i][1] - condition_sort[i][0] > condition[j][1] - condition[j][0] and condition_sort.count(condition[j])==False:
                    condition_sort[i] = condition[j]
        return condition_sort

condition = init()
sort(condition)
check(condition)
"""
3
2 4 3
<=
1 5 7
>=
3 4 2 
<=
"""
'''
4
2 4 3
<=
6 7 8 
<=
5 9 4
<=
7 8 9
<=
'''