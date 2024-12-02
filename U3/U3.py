import pickle
import heapq

# Načtení grafu - před spuštěním shapefile_to_graph
with open("graph.pkl", "rb") as f:
    graph = pickle.load(f)

print(f"Počet vrcholů: {graph.number_of_nodes()}")
print(f"Počet hran: {graph.number_of_edges()}")


class UnionFind:
    def __init__(self, nodes):
        self.parent = {node: node for node in nodes}
        self.rank = {node: 0 for node in nodes}

    def find(self, node):
        if self.parent[node] != node:
            self.parent[node] = self.find(self.parent[node])
        return self.parent[node]

    def union(self, u, v):
        root_u = self.find(u)
        root_v = self.find(v)
        if root_u != root_v:
            if self.rank[root_u] > self.rank[root_v]:
                self.parent[root_v] = root_u
            elif self.rank[root_u] < self.rank[root_v]:
                self.parent[root_u] = root_v
            else:
                self.parent[root_v] = root_u
                self.rank[root_u] += 1


# Kruskal algorithm
def kruskal(graph):
    edges = []
    for u in graph:
        for v, weight in graph[u]:
            edges.append((weight, u, v))
    edges.sort()

    uf = UnionFind(graph.keys())
    mst = []

    for weight, u, v in edges:
        if uf.find(u) != uf.find(v):
            uf.union(u, v)
            mst.append((u, v, weight))

    return mst


# Převede graf na adjacency list, kvůli využití vlastních algoritmů místo nx knihovny
def graph_to_adj_list(graph):
    adj_list = {}
    for u, v, data in graph.edges(data=True):
        weight = data.get('weight', 1)
        if u not in adj_list:
            adj_list[u] = []
        if v not in adj_list:
            adj_list[v] = []
        adj_list[u].append((v, weight))
        adj_list[v].append((u, weight))
    return adj_list


# Dijkstra algorithm
def dijkstra(graph, start, end):
    dist = {node: float('inf') for node in graph}
    dist[start] = 0
    prev = {node: None for node in graph}
    pq = [(0, start)]

    while pq:
        current_dist, current_node = heapq.heappop(pq)
        if current_node == end:
            break
        if current_dist > dist[current_node]:
            continue
        for neighbor, weight in graph[current_node]:
            distance = current_dist + weight
            if distance < dist[neighbor]:
                dist[neighbor] = distance
                prev[neighbor] = current_node
                heapq.heappush(pq, (distance, neighbor))

    path = []
    current = end
    while current is not None:
        path.insert(0, current)
        current = prev[current]

    return path, dist[end]


# BF algorithm
def bellman_ford(graph, start, end):
    dist = {node: float('inf') for node in graph}
    dist[start] = 0
    prev = {node: None for node in graph}

    for _ in range(len(graph) - 1):
        for u in graph:
            for v, weight in graph[u]:
                if dist[u] + weight < dist[v]:
                    dist[v] = dist[u] + weight
                    prev[v] = u

    for u in graph:
        for v, weight in graph[u]:
            if dist[u] + weight < dist[v]:
                raise ValueError("Graph contains a negative weight cycle.")

    path = []
    current = end
    while current is not None:
        path.insert(0, current)
        current = prev[current]

    return path, dist[end]


# Floyd-Warshall algorithm
def floyd_warshall(graph):
    nodes = list(graph.keys())
    dist = {u: {v: float('inf') for v in nodes} for u in nodes}
    for u in nodes:
        dist[u][u] = 0
    for u in graph:
        for v, weight in graph[u]:
            dist[u][v] = weight
    for k in nodes:
        for i in nodes:
            for j in nodes:
                dist[i][j] = min(dist[i][j], dist[i][k] + dist[k][j])
    return dist


# Prim
def prim(graph):
    start_node = next(iter(graph))
    visited = set()
    mst = []
    pq = [(0, start_node, None)]

    while pq:
        weight, current, previous = heapq.heappop(pq)
        if current in visited:
            continue
        visited.add(current)
        if previous is not None:
            mst.append((previous, current, weight))
        for neighbor, edge_weight in graph[current]:
            if neighbor not in visited:
                heapq.heappush(pq, (edge_weight, neighbor, current))

    return mst


# Kontroluje, zda použít Dijkstru, nebo BF
def has_negative_weights(graph):
    for edges in graph.values():
        for _, weight in edges:
            if weight < 0:
                return True
    return False


if __name__ == '__main__':
    # Pipeline
    adj_list = graph_to_adj_list(graph)

    # Úloha 1+2: Najít nejkratší cestu
    print("\nÚloha 1+2 - Hledání nejkratší cesty")
    start_node, end_node = 0, 10  # Replace with valid nodes
    if has_negative_weights(adj_list):
        print("Použit Bellman-Ford (záporné váhy detekovány)")
        path, length = bellman_ford(adj_list, start_node, end_node)
    else:
        print("Použit Dijkstra")
        path, length = dijkstra(adj_list, start_node, end_node)
    print(f"Nejkratší cesta mezi uzly {start_node} a {end_node}: {path} s délkou {length}")

    # Úloha 3: Krátké cesty mezi všemi dvojicemi
    print("\nÚloha 3 - Krátké cesty mezi všemi dvojicemi uzlů")
    all_pairs_shortest_paths = floyd_warshall(adj_list)
    print("Výpis částí výsledku Floyd-Warshall:")
    for start, targets in list(all_pairs_shortest_paths.items())[:5]:
        print(f"Cesty z uzlu {start}: {dict(list(targets.items())[:5])}")

    # Úloha 4: Minimální kostra (Prim)
    print("\nÚloha 4 - Minimální kostra (Prim)")
    mst_prim = prim(adj_list)
    print("Prim: Minimální kostra grafu:")
    for edge in mst_prim:
        print(edge)

    # Úloha 5: Minimální kostra (Kruskal)
    print("\nÚloha 5 - Minimální kostra (Kruskal)")
    mst_kruskal = kruskal(adj_list)
    print("Kruskal: Minimální kostra grafu:")
    for edge in mst_kruskal:
        print(edge)
