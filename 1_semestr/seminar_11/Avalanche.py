Heap_output = list(map(int,input().split()))
true = 0
def MaxHeap(Heap=[]):
    w=0
    for i in range(len(Heap)):
        try:
            if Heap[i] > Heap[2*i+1] and Heap[i] > Heap[2*i+2]:
                w+=1
            else:
                return 0
        except:
            return 1
while true!=1:
    Center = len(Heap_output)//2+1
    shit = Center-1
    Flag = True
    if shit<=0:
        Flag = False
        if Heap_output[0]<Heap_output[-1]:
            print(Heap_output[-1],Heap_output[0])
        else:
            print(*Heap_output)
    if Flag:
         for i in Heap_output[-Center::-1]:
            shit-=1
            if 2*shit+2< len(Heap_output)-1:
                if i < Heap_output[2*shit+1] or i < Heap_output[2*shit+2]:
                    if Heap_output[2*shit+1] > Heap_output[2*shit+2]:
                        Heap_output[shit],Heap_output[2*shit+1] = Heap_output[2*shit+1],Heap_output[shit]
                    elif Heap_output[2*shit+1] < Heap_output[2*shit+2]:
                        Heap_output[shit], Heap_output[2 * shit + 2] = Heap_output[2 * shit + 2], Heap_output[shit]

            else:
                if i < Heap_output[2 * shit + 1]:
                    Heap_output[shit], Heap_output[2 * shit + 1] = Heap_output[2 * shit + 1], Heap_output[shit]
    true=MaxHeap(Heap_output)

print(*Heap_output)