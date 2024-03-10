class Graph:
    def __init__(self, vertices):
        self.graph = defaultdict(list)  # dictionary containing adjacency List
        self.V = vertices  # No. of vertices

    # Функция для добавления ребра в граф
    def addEdge(self, u, v):
        self.graph[u].append(v)

    # Рекурсивная функция, используемая topologicalSort
    def topologicalSortUtil(self, v, visited, stack):

        # Помечаем текущий узел как посещенный
        visited[v] = True

        # Рекурсивно вызываем функцию для всех смежных вершин
        for i in self.graph[v]:
            if visited[i] == False:
                self.topologicalSortUtil(i, visited, stack)

        # Добавляем текущую вершину в стек с результатом
        stack.insert(0, v)

    # Функция для поиска топологической сортировки.
    # Рекурсивно использует topologicalSortUtil()
    def topologicalSort(self):
        # Помечаем все вершины как непосещенные
        visited = [False] * self.V
        stack = []

        # Вызываем рекурсивную вспомогательную функцию
        # для поиска топологической сортировки для каждой вершины
        for i in range(self.V):
            if visited[i] == False:
                self.topologicalSortUtil(i, visited, stack)

        # Выводим содержимое стека
        print (stack)


g = Graph(6)
g.addEdge(5, 2);
g.addEdge(5, 0);
g.addEdge(4, 0);
g.addEdge(4, 1);
g.addEdge(2, 3);
g.addEdge(3, 1);

print ("Following is a Topological Sort of the given graph")
g.topologicalSort()