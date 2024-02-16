class Transformer:
    def __init__(self):
        self.x = 0
        self.m = 0
        self.name = ''
    def add(self,x,m, name):
        self.x = x
        self.m = m
        self.name = name
    def repr(self):
        print('Находиться на позиции: '+ str(self.x))
        print('Масса: ' + str(self.m))
        print('Имя: ' + self.name)
    def run(self):
        self.x+=1
    def massa(self):
        if self.m> 0:
            self.m-=1
        else:
            return 0
    def fight(self, name):
        name.massa()
        if name.m ==0:
            print(name.name + ' проиграл битву!')
            return 0

Optimus = Transformer()
Optimus.add( 1, 100,'Optimus')
Triticon = Transformer()
Triticon.add(3,5, "Triticon")
Optimus.repr()
Triticon.fight(Optimus)
for i in range(50):
    Optimus.run()
Triticon.fight(Optimus)
for i in range(10):
    Optimus.fight(Triticon)
    if Optimus.fight(Triticon) == 0:
        break
Triticon.repr()
Optimus.repr()