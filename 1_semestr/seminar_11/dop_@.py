class MaxHeap():
    def __init__(self,items = []):
        self.heap = [0]
        for i in items:
            self.heap.append(i)
            self.__floatUp(len(self.heap)-1)
    def __swap(self,i,j):
        self.heap[i],self.heap[j]=self.heap[j],self.heap[i]
    def __floatUp(self,index):
        parent = index//2
        if index <=1:
            return
        elif self.heap[index] > self.heap[parent]:
            self.__swap(index, parent)
            self.__floatUp(parent)
    def __bubbleDown(self,index):
        left = index * 2
        right = index*2 +1
        largest = index
        if len(self.heap)>left and self.heap[largest]<self.heap[left]:
            largest = left
        if len(self.heap)>right and self.heap[largest]<self.heap[right]:
            largest = right
        if largest != index:
            self.__swap(index, largest)
            self.__bubbleDown(largest)
    def __repr__(self):
        return str(self.heap[1:len(self.heap)])
m = MaxHeap(list(map(int, input().split())))
print(m)
