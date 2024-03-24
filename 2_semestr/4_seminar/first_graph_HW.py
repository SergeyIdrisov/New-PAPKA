
def read_graph_as_edges():
    n = int(input())
    graph = [list(map(int, input().split())) for i in range(n)]
    # for i in range(n):
    #     graph.append(list(map(int, input().split())))
    return  graph
def read_graph_as_neigh_list():
    edge_list = read_graph_as_edges()
    graph_dict = {} #dict()
    vertex_set = set()
    for edge in edge_list:
        vertex_set.add(edge[0])
        vertex_set.add(edge[1])
    V_num = len(vertex_set)
    for v in vertex_set:
        graph_dict[v] = frozenset()
    for edge in edge_list:
        if edge[0] not in graph_dict.keys():
            graph_dict[edge[0]] = frozenset([edge[1]])
        else:
            graph_dict[edge[0]]= graph_dict[edge[0]] | frozenset([edge[1]])
    return graph_dict

def read_graph_as_neigh_matrix():
    edge_list = read_graph_as_edges()
    vertex_set = set()
    for edge in edge_list:
        vertex_set.add(edge[0])
        vertex_set.add(edge[1])
    V_num = len(vertex_set)


    res_matrix = [[0 for i in range(V_num)]for j in range(V_num)]
    for edge in edge_list:
        index_1 = edge[0] -1
        index_2 = edge[1]-1
        res_matrix[index_1][index_2] = 1

    return res_matrix
def print_matrix(matrix):
    for line in matrix:
        print(*line)
def DFS(graph, v, visited=[]):
    #print(v)
    visited.append(v)
    for neigh in graph[v]:
        if neigh not in visited:
            DFS(graph, neigh, visited)

def isCyclicUtil(v, visited, recStack, graph):
    visited[v] = True
    recStack[v] = True

    for neighbour in graph[v]:
        if visited[neighbour] == False:
            if isCyclicUtil(neighbour, visited, recStack, graph) == True:
                return True
        elif recStack[neighbour] == True:
            return True

    recStack[v] = False
    return False


def has_cycle( graph):
    V=len(graph)
    visited = [False] * (V + 2)
    recStack = [False] * (V + 2)
    for node in range(V):
        if visited[node] == False:
            if isCyclicUtil(node+1, visited, recStack, graph) == True:
                return True
    return False
def topologicalSortUtil(v, visited, stack = [], i = 1):
    visited[v] = True
    for v in graph:
        if visited[i]== False:
            topologicalSortUtil(i, visited, stack)
            stack.append(v)
        i+=1


def topologicalSort(graph):
    if has_cycle(graph) == False:
        visited = [False] * (len(graph)+1)
        stack = []
        for i in range(len(graph)):
            if visited[i] == False:
                topologicalSortUtil(i, visited, stack)
        return stack
    else:
        t ='Невозможно выполнить из-за присутсвия цикла в данном графе...'
        return t
def road_in_v_from_u(graph, v, u):
    Topolog_list = topologicalSort(graph)
    if type(Topolog_list) == str:
        return Topolog_list
    else:
        tops = {v: 0 for v in graph}
        tops[v]=1
        for i in Topolog_list[::-1]:
            for j in graph[i]:
                tops[j] += tops[i]
        return tops[u]

def father_n(graph, v , u):
    result = road_in_v_from_u(graph, v , u )
    if result > 0:
        return True
    else:
        return False

def bfs(graph, v):
    visited = []
    queue = []
    d = {}
    for keys in graph.keys():
        d[keys] = 100000
    visited.append(v)
    queue.append(v)
    d[v] = 0

    while queue:
        u = queue.pop(0)
        print(u, end = " ")

        for neighdour in graph[u]:
            if neighdour not in visited:
                visited.append(neighdour)
                queue.append(neighdour)
                d[neighdour] = d[u] + 1
    return d


graph = read_graph_as_neigh_list()
#DFS(graph, 1)
print(has_cycle(graph))
#print(has_cycle(graph))
#print(topologicalSort(graph))
#print(graph)
#print(road_in_v_from_u(graph, 1, 7))
#print(father_n(graph, 6 , 7))
#d = bfs(graph, 1)
#print(d)
graph_1 = read_graph_as_neigh_matrix()
print(graph_1)

'''
8
1 4 
1 2 
3 2 
2 5 
2 6 
5 3 
6 2 
6 4 
'''
'''
10
1 4 6
1 2 1
3 2 2
2 5 3
2 6 1
5 3 1
6 2 2
6 4 1
7 8
8 7
'''
'''
7
1 3
1 5
1 6
1 7
3 4
4 7
6 7
'''

'''
6
5 2
5 0
4 0
4 1
2 3
3 1
'''
'''
6
1 8 
1 67
67 5
67 6
5 12
1 5
'''
'''
4
1 2
1 3
3 4
2 4

'''

'''
9
1 2
1 3
2 3
3 4
4 2
4 5
5 6
6 1
6 3
'''