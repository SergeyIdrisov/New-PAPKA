class Vector():
    def __init__(self,x,y,z):
            self.x = x
            self.y = y
            self.z = z
    def __abs__(self):
            return (self.x**2+self.y**2+self.z**2)**0.5
    def __add__(self, other):
        if isinstance(other, Vector) == True:
            return Vector(self.x+other.x,self.y+other.y,self.z+other.z)
        else:
            return Vector(self.x+other,self.y+other,self.z+other)

    def __radd__(self, other):
        if isinstance(other, Vector) == True:
            return Vector(self.x + other.x, self.y + other.y, self.z + other.z)
        else:
            return Vector(self.x + other, self.y + other, self.z + other)
    def __sub__(self, other):
        if isinstance(other, Vector) == True:
            return Vector(self.x-other.x,self.y-other.y,self.z-other.z)
        else:
            return Vector(self.x-other,self.y-other,self.z-other)
    def __rsub__(self, other):
        if isinstance(other, Vector) == True:
            return Vector(self.x-other.x,self.y-other.y,self.z-other.z)
        else:
            return Vector(self.x-other,self.y-other,self.z-other)
    def __mul__(self, other):
        if isinstance(other, Vector) == True:
            return Vector(self.x*other.x,self.y*other.y,self.z*other.z)
        else:
            return Vector(self.x*other,self.y*other,self.z*other)
    def __rmul__(self, other):
        if isinstance(other, Vector) == True:
            return Vector(self.x*other.x,self.y*other.y,self.z*other.z)
        else:
            return Vector(self.x*other,self.y*other,self.z*other)
    def __truediv__(self, other):
        if isinstance(other, Vector) == True:
            return Vector(self.x/other.x,self.y/other.y,self.z/other.z)
        else:
            return Vector(self.x/other,self.y/other,self.z/other)

    def __rtruediv__(self, other):
        if isinstance(other, Vector) == True:
            return Vector(self.x / other.x, self.y / other.y, self.z / other.z)
        else:
            return Vector(self.x / other, self.y / other, self.z / other)
    def vector_proz(self,other):
        x = self.y*other.z - self.z*self.y
        y = -(self.x*self.z-self.z*other.x)
        z = self.x*other.y - self.y*other.x
        return Vector(x,y,z)
    #x y z
    #1 2 3
    #4 5 6
    def __repr__(self):
        return f'({self.x}, {self.y}, {self.z})'
N = list(map(float, input().split()))
v=[0]*int(len(N)/3)
for i in range(int(len(N)/3)):
    v[i] = Vector(N[i*3],N[i*3+1],N[i*3+2])
b = []
ind = []
for a in v:
    b.append(abs(a))
for i in range(2):
     ind.append(b.index(max(b)))
     b.pop(b.index(max(b)))

ans = abs(Vector.vector_proz(v[ind[0]],v[ind[-1]]))/2
print(ans)
