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
    def __repr__(self):
        return f'({self.x}, {self.y}, {self.z})'

N = list(map(float, input().split()))
v1 = Vector(N[0],N[1],N[2])
v2 = Vector(N[3], N[4], N[5])
print(v1, '- v1')
print(v2, '- v2')
print(v1 + 5, '- v1 + 5')
print(v1*5, '- v1*5')
print(v1*5 + 5 , '- v1*5 + 5')
print(v1*v2 , '- v1*v2')