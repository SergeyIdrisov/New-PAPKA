class matrix():
    def __init__(self,a,b):
            self.a = a
            self.b = b
    def matr_poz(self, other):
        if self.b == other.a:
            a = self.a
            b = other.b
            return (matrix(a,b))
        else:
            return print('error')
    def poik_sum(self,other):
        if self.b == other.a:
            sum = self.a*self.b*other.b
            return sum
        else:
            return 'error'
    def __repr__(self):
        return f'({self.a}, {self.b})'
shit = int(input())
B= [0]*shit
zadachi = []
for i in range(shit):
    peremenaya = list(input().split())
    B[i] = [int(peremenaya[1]),int(peremenaya[2])]
while shit != 0:
    zadachi += list(map(str, input().split()))
    if zadachi[-1]=='0':
        shit = 0
        zadachi.pop(-1)
for i in zadachi:
    if len(i)<=1:
        print(0)
    else:
        for j in range(len(i)):
            

print(B[0])
print(matrix.poik_sum(matrix(B[0][0],B[0][1]),matrix(B[1][0],B[1][1])))
print(zadachi)
