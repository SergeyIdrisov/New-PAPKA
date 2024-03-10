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

def has_cycle(graph, v, visited = []):
    result = False
    visited.append(v)
    for neigh in graph[v]:

        if neigh in visited:
            result = True
            return result

        if result == False:
              result = has_cycle(graph, neigh, visited)

    return result
def topologicalSortUtil(v, visited, stack = [], i = 1):
    visited[v] = True
    for v in graph:
        if visited[i]== False:
            topologicalSortUtil(i, visited, stack)
            stack.append(v)
        i+=1


def topologicalSort(graph):
    if has_cycle(graph, v=1) == False:
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
        First_index =Topolog_list.index(v)
        Second_index = Topolog_list.index(u)
        answer = 0
        visited = [v]
        if First_index < Second_index:
            return answer
        else:
            for i in graph:
                if i in visited:
                    answer+=1
                else:
                    pass
                visited.append(i)
            return answer

def father_n(graph, v , u):
    result = road_in_v_from_u(graph, v , u )
    if result > 0:
        return True
    else:
        return False


graph = read_graph_as_neigh_list()
DFS(graph, 1)
#print(has_cycle(graph, 1))
#print(topologicalSort(graph))
#print(graph)
#print(road_in_v_from_u(graph, 7, 6))
print(father_n(graph, 6 , 7))

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
7
1 3
1 5
1 7
3 4
6 2
6 7
6 8
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