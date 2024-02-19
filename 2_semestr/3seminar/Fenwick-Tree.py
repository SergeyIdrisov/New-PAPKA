




class Fenwick_Tree():
    def __init__ (self, data: list):
        self.data = data
        self.prefix_sum = [0]*(len(data)+1)
        i = 0
        while i < len(self.data):
            self.prefix_sum[i] = self.prefix_sum[i-1] + self.data[i]
            i+=1


    def sum(self,l:int, r:int):
        return self.prefix_sum[r]-self.prefix_sum[l-1]
    def update(self, i , volume):
        self.data[i] += volume
        while i < len(self.data):
            self.prefix_sum[i] = self.prefix_sum[i - 1] + self.data[i]
            i += 1
        return self.prefix_sum



tree = Fenwick_Tree([1,2,3,4,5,6,7,8,57,5,48,48,48,48,4,84,84,84,84,84,84,64,51,3,457,21,4,74,54,98,5])

print(Fenwick_Tree.sum(tree,5,10))

Fenwick_Tree.update(tree, 5, -20)

print(Fenwick_Tree.sum(tree, 5, 10 ))

