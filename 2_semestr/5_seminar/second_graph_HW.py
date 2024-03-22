def read_graph_as_edges_w():
    n = int(input())
    graph = [list(map(int, input().split())) for i in range(n)]
    # for i in range(n):
    #     graph.append(list(map(int, input().split())))
    return graph


def read_graph_as_neigh_list_w():
    edge_list = read_graph_as_edges_w()
    graph_dict = {}  # dict()
    vertex_set = set()
    for edge in edge_list:
        vertex_set.add(edge[0])
        vertex_set.add(edge[1])
    for v in vertex_set:
        graph_dict[v] = frozenset()
    for edge in edge_list:
            graph_dict[edge[0]] = graph_dict[edge[0]] | frozenset([(edge[1],edge[2])])
    return graph_dict


def read_graph_as_neigh_matrix_w():
    edge_list = read_graph_as_edges_w()
    vertex_set = set()
    for edge in edge_list:
        vertex_set.add(edge[0])
        vertex_set.add(edge[1])
    V_num = len(vertex_set)

    res_matrix = [[0 for i in range(V_num)] for j in range(V_num)]
    for edge in edge_list:
        index_1 = edge[0] - 1
        index_2 = edge[1] - 1
        res_matrix[index_1][index_2] = edge[2]

    return res_matrix


def print_matrix_w(matrix):
    for line in matrix:
        print(*line)


def DFS_w(graph, v, visited=[]):
    # print(v)
    visited.append(v)
    for neigh in graph[v]:
        if neigh not in visited:
            DFS_w(graph, neigh, visited)


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
            if isCyclicUtil(neighbour, visited, recStack, graph) == True:
                return True
        elif recStack[neighbour] == True:
            return True

    # The node needs to be popped from
    # recursion stack before function ends
    recStack[v] = False
    return False

def has_cycle_w( graph):
    V=len(graph)
    visited = [False] * (V + 2)
    recStack = [False] * (V + 2)
    for node in range(V):
        if visited[node] == False:
            if isCyclicUtil(node+1, visited, recStack, graph) == True:
                return True
    return False


def topologicalSortUtil_w(v, visited, stack=[], i=1):
    visited[v] = True
    for v in graph:
        if visited[i] == False:
            topologicalSortUtil_w(i, visited, stack)
            stack.append(v)
        i += 1


def topologicalSort_w(graph):
    if has_cycle_w(graph, v=1) == False:
        visited = [False] * (len(graph) + 1)
        stack = []
        for i in range(len(graph)):
            if visited[i] == False:
                topologicalSortUtil_w(i, visited, stack)
        return stack
    else:
        t = 'Невозможно выполнить из-за присутсвия цикла в данном графе...'
        return t


def road_in_v_from_u_w(graph, v, u):
    Topolog_list = topologicalSort_w(graph)
    if type(Topolog_list) == str:
        return Topolog_list
    else:
        tops = {v: 0 for v in graph}
        tops[v] = 1
        for i in Topolog_list[::-1]:
            for j in graph[i]:
                tops[j] += tops[i]
        # {1:1, 2:0, 3:0, 4:0} -> {}
        # {1: frozenset({2, 3}), 2: frozenset({4}), 3: frozenset({4}), 4: frozenset()}
        # [4,3,2,1]
        return tops[u]


def father_n_w(graph, v, u):
    result = road_in_v_from_u_w(graph, v, u)
    if result > 0:
        return True
    else:
        return False


def bfs_w(graph, v):
    visited = []
    queue = []
    d = {}
    for keys in graph.keys():
        d[keys] = float('infinity')
    visited.append(v)
    queue.append(v)
    d[v] = 0

    while queue:
        u = queue.pop(0)
        print(u, end=" ")

        for neighdour in graph[u]:
            if neighdour not in visited:
                visited.append(neighdour)
                queue.append(neighdour)
                d[neighdour] = d[u] + 1
    return d


def Dijkstra_string_concatenation(graph, v):
    strok = {}
    Dijkstra = Dijkstra_sum_min(graph, v)
    visited = []
    end = []
    for key in graph.keys():
        strok[key] = ''
    Dijkstra[v] = 0
    visited.append([0, v])
    while visited:
        visited.sort()
        c = visited.pop(0)
        end.append(c[1])
        for neigh in graph[c[1]]:
            if neigh[0] not in end:
                if Dijkstra[neigh[0]] >= neigh[1]:
                    strok[neigh[0]]=strok[c[1]]+str(neigh[1])
                visited.append(neigh[::-1])
    return strok



def Dijkstra_sum_min(graph, v):
    d = {}
    visited = []
    end = []
    for key in graph.keys():
        d[key] = float('infinity')

    d[v] = 0
    visited.append([0, v])
    while visited:
        visited.sort()

        c = visited.pop(0)
        end.append(c[1])
        for neigh in graph[c[1]]:
            if neigh[0] not in end:
                if (d[c[1]] + neigh[1]) < d[neigh[0]]:
                    d[neigh[0]] = (d[c[1]] + neigh[1])
                visited.append(neigh[::-1])

    return d

def Dijkstra_sum_max(graph, v):
    d = {}
    visited = []
    end = []
    for key in graph.keys():
        d[key] = 0

    d[v] = 0
    visited.append([0, v])
    while visited:
        visited.sort()

        c = visited.pop(0)
        end.append(c[1])
        for neigh in graph[c[1]]:
            if neigh[0] not in end:
                if (d[c[1]] + neigh[1]) > d[neigh[0]]:
                    d[neigh[0]] += (d[c[1]] + neigh[1])
                visited.append(neigh[::-1])

    return d

def Dijkstra_mult_min(graph, v):
    d = {}
    visited = []
    end = []
    for key in graph.keys():
        d[key] = float('infinity')

    d[v] = 1
    visited.append([0, v])
    while visited:
        visited.sort()

        c = visited.pop(0)
        end.append(c[1])
        for neigh in graph[c[1]]:
            if neigh[0] not in end:
                if (d[c[1]] + neigh[1]) < d[neigh[0]]:
                    d[neigh[0]] = (d[c[1]] * neigh[1])
                visited.append(neigh[::-1])

    return d

def Floyd_Warshall(graph):
        v = len(graph)
        d = [[float('infinity') for i in range(v)] for j in range(v)]
        nxt = [[-1 for i in range(v)] for j in range(v)]
        for i in range(v):
            for j in range(v):
                if graph[i][j] != 0:
                    d[i][j] = graph[i][j]
                    nxt[i][j] = j
        for k in range(1, v):
            for i in range(v):
                for j in range(v):
                    if d[i][k] + d[k][j] < d[i][j]:
                        d[i][j] = d[i][k] + d[k][j]
                        nxt[i][j] = nxt[i][k]
        return d, nxt

def pth(i, j, nxt):
    p = [i]
    while nxt[i-1][j-1]+1 != j:
        i = nxt[i-1][j-1]+1
        p.append(i)
    p.append(j)
    return p



#graph = read_graph_as_neigh_list_w()
graph = read_graph_as_neigh_matrix_w()
#DFS_w(graph, 1)
# print(has_cycle(graph, 1))
# print(topologicalSort(graph))
#print(graph)
#print_matrix_w(read_graph_as_neigh_matrix_w())
# print(road_in_v_from_u(graph, 7, 6))
# print(father_n(graph, 6 , 7))
#d = bfs_w(graph, 1)
#print(d)
#print(Dijkstra_sum_min(graph, 1))
#print(Dijkstra_sum_max(graph, 1))
#print(Dijkstra_mult_min(graph, 1))
#print(Dijkstra_string_concatenation(graph, 1))
#D, P = Floyd_Warshall(graph)
#print(pth(1, 3, P))

'''
8
1 4 6
1 2 1
3 2 2
2 5 3
2 6 1
5 3 1
6 2 2
6 4 1
'''

'''
5
1 2 1
2 4 1
4 3 1
2 3 4
1 3 6
'''

'''
6
1 2 -1 
2 1 -1 
1 3 -1
2 3 -1 
3 1 -1
3 2 -1
'''