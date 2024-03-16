def isCyclicUtil(v, visited, recStack, graph):
    # Mark current node as visited and
    # adds to recursion stack
    visited[v] = True
    recStack[v] = True

    # Recur for all neighbours
    # if any neighbour is visited and in
    # recStack then graph is cyclic
    for neighbour in graph[v]:
        if visited[neighbour] == False:
            if isCyclicUtil(neighbour, visited, recStack) == True:
                return True
        elif recStack[neighbour] == True:
            return True

    # The node needs to be popped from
    # recursion stack before function ends
    recStack[v] = False
    return False


# Returns true if graph is cyclic else false
def isCyclic(V, graph):
    visited = [False] * (V + 1)
    recStack = [False] * (V + 1)
    for node in range(V):
        if visited[node] == False:
            if isCyclicUtil(node, visited, recStack, graph) == True:
                return True
    return False