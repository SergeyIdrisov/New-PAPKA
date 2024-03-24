class Quation:
    def check(self, left, right):
        pass
    def init(self):
        self.znak = []
        self.condition = []
        self.q = int(input())
        for i in range(self.q):
            self.condition += list(map(int, input()))
            self.znak += list(input())
condition = Quation
print(condition.init())
"""
3
2 4 3
<=
1 5 7
>=
3 4 2
<=
"""