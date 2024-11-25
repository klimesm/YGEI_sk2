# Imports
from math import inf
from queue import PriorityQueue

# Functions definition

# Reconstruct the path from predecessors
def rec_path(u, v, P):
    path = []
    while v != u and v != -1:
        path.append(v)
        v = P[v]
    path.append(v)
    path.reverse()
    return path

# Dijkstra's algorithm
def dijkstra(G, start, end):
    n = len(G)
    P = [-1] * (n + 1)
    D = [inf] * (n + 1)
    PQ = PriorityQueue()
    PQ.put((0, start))
    D[start] = 0

    while not PQ.empty():
        du, u = PQ.get()
        if u == end:
            break
        for v, wuv in G[u].items():
            if D[v] > D[u] + wuv:
                D[v] = D[u] + wuv
                P[v] = u
                PQ.put((D[v], v))

    return P, D[end]

# Bellman-Ford algorithm for negative weights - if the loop sum is negative, there is no least-cost path (stacking)
def bellman_ford(G, start, end):
    n = len(G)
    D = [inf] * (n + 1)
    P = [-1] * (n + 1)
    D[start] = 0

    for _ in range(n - 1):
        for u in G:
            for v, w in G[u].items():
                if D[u] != inf and D[u] + w < D[v]:
                    D[v] = D[u] + w
                    P[v] = u

    for u in G:
        for v, w in G[u].items():
            if D[u] != inf and D[u] + w < D[v]:
                raise ValueError("Graph contains a negative-weight cycle")

    return P, D[end]

# Find shortest path using either Dijkstra or Bellman-Ford
def find_shortest_path(G, start, end):
    has_negative_weight = any(w < 0 for edges in G.values() for w in edges.values())

    if has_negative_weight:
        print("Negative weights detected. Using Bellman-Ford algorithm.")
        try:
            P, distance = bellman_ford(G, start, end)
        except ValueError as e:
            print(e)
            return None, None
    else:
        print("No negative weights detected. Using Dijkstra's algorithm.")
        P, distance = dijkstra(G, start, end)

    return P, distance

# All pairs shortest paths using find_shortest_path
def all_pairs_shortest_paths(G):
    nodes = list(G.keys())
    shortest_paths = {u: {v: None for v in nodes} for u in nodes}
    distances = {u: {v: float('inf') for v in nodes} for u in nodes}

    for u in nodes:
        for v in nodes:
            if u == v:
                shortest_paths[u][v] = [u]
                distances[u][v] = 0
            else:
                P, dist = find_shortest_path(G, u, v)
                if P and dist is not None:
                    shortest_paths[u][v] = rec_path(u, v, P)
                    distances[u][v] = dist

    return shortest_paths, distances

# Find operation for Kruskal
def find(u, P):
    if P[u] != u:
        P[u] = find(P[u], P)
    return P[u]

# Union operation for Kruskal
def union(u, v, P, rank):
    root_u = find(u, P)
    root_v = find(v, P)
    if root_u != root_v:
        if rank[root_u] > rank[root_v]:
            P[root_v] = root_u
        elif rank[root_u] < rank[root_v]:
            P[root_u] = root_v
        else:
            P[root_v] = root_u
            rank[root_u] += 1

# Kruskal's algorithm for MSF
def kruskal(V, E):
    P = {v: v for v in V}
    rank = {v: 0 for v in V}
    E_sorted = sorted(E, key=lambda edge: edge[2])
    MST = []
    total_weight = 0

    for u, v, w in E_sorted:
        if find(u, P) != find(v, P):
            union(u, v, P, rank)
            MST.append((u, v, w))
            total_weight += w

    return MST, total_weight

# Prim's algorithm for MSF
def prim_msf(G):
    nodes = list(G.keys())
    visited = set()
    msf = []
    total_weight = 0

    def prim(start):
        nonlocal total_weight
        pq = PriorityQueue()
        visited.add(start)
        for neighbor, weight in G[start].items():
            pq.put((weight, start, neighbor))
        mst_edges = []
        while not pq.empty():
            weight, u, v = pq.get()
            if v not in visited:
                visited.add(v)
                mst_edges.append((u, v, weight))
                total_weight += weight
                for neighbor, edge_weight in G[v].items():
                    if neighbor not in visited:
                        pq.put((edge_weight, v, neighbor))
        return mst_edges

    for node in nodes:
        if node not in visited:
            mst_edges = prim(node)
            msf.extend(mst_edges)

    return msf, total_weight


# Define Graph, Vertices, and Edges
G = {
    1: {2: 8, 3: 4, 5: 2},
    2: {1: 8, 3: 5, 4: 2, 7: 6, 8: 7},  # Negative weight (-2) from 2 to 4
    3: {1: 4, 2: 5, 6: 3, 7: 4},
    4: {2: 2, 9: 3},  # Negative weight (-2) from 4 to 2
    5: {1: 2, 6: 5},
    6: {3: 3, 5: 5, 7: 5, 8: 7, 9: 10},
    7: {2: 6, 3: 4, 6: 5, 8: 3},
    8: {2: 7, 6: 7, 7: 3, 9: 1},
    9: {4: 3, 6: 10, 8: 1}  # No negative cycle
}


V = [1, 2, 3, 4, 5, 6, 7, 8, 9]

E = [
    (1, 2, 8),
    (1, 3, 4),
    (1, 5, 2),
    (2, 3, 5),
    (2, 4, 2),
    (2, 7, 6),
    (2, 8, 7),
    (3, 6, 3),
    (3, 7, 4),
    (4, 9, 3),
    (5, 6, 5),
    (6, 7, 5),
    (6, 8, 7),
    (6, 9, 10),
    (7, 8, 3),
    (8, 9, 1)
]

# Example: Find shortest path
start, end = 1, 9
P, distance = find_shortest_path(G, start, end)
if P and distance is not None:
    path = rec_path(start, end, P)
    print("\nFind shortest path")
    print("Path:", path)
    print("Distance:", distance)

# Example: All pairs shortest paths
shortest_paths, distances = all_pairs_shortest_paths(G)
print("\nAll pairs shortest paths")
print("Shortest distances:", distances)

# Example: Kruskal's algorithm
MST, total_weight = kruskal(V, E)
print("\nMinimum Spanning Tree using Kruskal's algorithm")
print("Edges:", MST)
print("Total weight:", total_weight)

# Example: Prim's algorithm for MSF
msf, total_weight = prim_msf(G)
print("\nMinimum Spanning Forest using Prim's algorithm")
print("Edges:", msf)
print("Total weight:", total_weight)
