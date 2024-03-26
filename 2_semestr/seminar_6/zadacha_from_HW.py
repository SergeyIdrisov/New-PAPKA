
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
    condition = condition[0]
    for i in range(len(znak)-1):
        if znak[i] ==znak[i+1] or (znak[i] == '<=' and znak[i+1] == '>='):
            return True
        else:
            pass
def sort(condition):
        znak = condition[-1]
        condition = condition[0]
        znak_sort = ['']* len(znak)
        q = len(condition)
        condition_sort = [[1, 100000000000, 0] for _ in range(q)]
        for i in range(q):
            for j in range(q):
                if condition_sort[i][1] - condition_sort[i][0] > condition[j][1] - condition[j][0] and condition_sort.count(condition[j])==False:
                    condition_sort[i] = condition[j]
                    znak_sort[i] = znak[j]
        condition_sort.append(znak_sort)
        return condition_sort

condition = init()
print(sort(condition))
print(check(condition))
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