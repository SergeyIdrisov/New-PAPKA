




class Fenwick_Tree():
    def __init__ (self, data: list):
        self.data = data
        self.prefix_sum = [0]*(len(data)+1)
        i = 0
        while i < len(self.data):
            self.prefix_sum[i] = self.prefix_sum[i-1] + self.data[i]
            i+=1
            print(self.prefix_sum)


    def sum(self,l:int, r:int):
        return self.prefix_sum[r]-self.prefix_sum[l-1]
    def update(self, i , volume):
        self.data[i] = volume
        while i < len(self.data):
            self.prefix_sum[i] = self.prefix_sum[i - 1] + self.data[i]
            i += 1
            print(self.prefix_sum)
        return self.prefix_sum



tree = Fenwick_Tree([1,2,3,4,5,6,7,8])

print(Fenwick_Tree.sum(tree,0,6))

Fenwick_Tree.update(tree, 5, 3)

print(Fenwick_Tree.sum(tree, 0, 6 ))

