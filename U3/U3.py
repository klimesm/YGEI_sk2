import pickle
import heapq
import networkx as nx

# Load the graph
with open("graph.pkl", "rb") as f:
    graph = pickle.load(f)

print(f"Number of nodes: {graph.number_of_nodes()}")
print(f"Number of edges: {graph.number_of_edges()}")

# Create mappings for names and vertex indices
name_to_vertex = {data['name']: node for node, data in graph.nodes(data=True) if data.get('is_settlement')}
vertex_to_name = {node: data['name'] for node, data in graph.nodes(data=True) if data.get('is_settlement')}


print("Name-to-Vertex Map:", name_to_vertex)

# Extract the vertex-to-name mapping directly from node attributes
vertex_to_name = {node: name for node, name in nx.get_node_attributes(graph, 'name').items() if name is not None}

# Print to verify the mapping
print("Vertex-to-Name Mapping:")
for vertex, name in vertex_to_name.items():
    print(f"Vertex {vertex}: {name}")

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


# Resolve node index from name or index
def resolve_node(node):
    """
    Resolves a node name to its vertex index or returns the node directly if it is already an index.

    Parameters:
    - node: str (name) or int (vertex index)

    Returns:
    - int: vertex index
    """
    if isinstance(node, str):  # Node is a settlement name
        # Reverse vertex-to-name to create name-to-vertex on demand
        name_to_vertex = {name: vertex for vertex, name in vertex_to_name.items() if name}

        if node in name_to_vertex:
            return name_to_vertex[node]  # Correctly resolve the vertex index
        else:
            raise ValueError(f"Settlement '{node}' not found in the graph.")
    return node  # Return the node directly if it's already an integer



# Kruskal algorithm
def kruskal(graph):
    edges = []
    for u, v, data in graph.edges(data=True):
        weight = data.get('weight', 1)
        edges.append((weight, u, v))
    edges.sort()

    uf = UnionFind(graph.nodes)
    mst = []

    for weight, u, v in edges:
        if uf.find(u) != uf.find(v):
            uf.union(u, v)
            mst.append((u, v, weight))

    return mst


# Convert graph to adjacency list
def graph_to_adj_list(graph, use_length_as_weight=False):
    adj_list = {}
    for u, v, data in graph.edges(data=True):
        weight = data['length'] if use_length_as_weight else data.get('weight', 1)
        if u not in adj_list:
            adj_list[u] = []
        if v not in adj_list:
            adj_list[v] = []
        adj_list[u].append((v, weight))
        adj_list[v].append((u, weight))
    return adj_list


# Dijkstra algorithm
def dijkstra(graph, start, end):
    """Custom Dijkstra algorithm for shortest path."""
    start = resolve_node(start)
    end = resolve_node(end)

    dist = {node: float('inf') for node in graph}
    dist[start] = 0
    prev = {node: None for node in graph}
    pq = [(0, start)]  # Priority queue: (distance, node)

    while pq:
        current_dist, current_node = heapq.heappop(pq)

        if current_node == end:
            break

        if current_dist > dist[current_node]:
            continue

        for neighbor, weight in graph[current_node]:  # Use adj_list structure
            distance = current_dist + weight
            if distance < dist[neighbor]:
                dist[neighbor] = distance
                prev[neighbor] = current_node
                heapq.heappush(pq, (distance, neighbor))

    # Reconstruct path
    path = []
    current = end
    while current is not None:
        path.insert(0, current)
        current = prev[current]

    return path, dist[end]



# Bellman-Ford algorithm
def bellman_ford(graph, start, end):
    start = resolve_node(start)
    end = resolve_node(end)

    dist = {node: float('inf') for node in graph}
    dist[start] = 0
    prev = {node: None for node in graph}

    for _ in range(len(graph) - 1):
        for u, v, data in graph.edges(data=True):
            weight = data.get('weight', 1)
            if dist[u] + weight < dist[v]:
                dist[v] = dist[u] + weight
                prev[v] = u

    # Check for negative weight cycles
    for u, v, data in graph.edges(data=True):
        weight = data.get('weight', 1)
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
    for u, v, data in graph.edges(data=True):
        weight = data.get('weight', 1)
        dist[u][v] = weight
    for k in nodes:
        for i in nodes:
            for j in nodes:
                dist[i][j] = min(dist[i][j], dist[i][k] + dist[k][j])
    return dist


# Prim's algorithm
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
        for neighbor in graph.neighbors(current):
            edge_weight = graph[current][neighbor].get('weight', 1)
            if neighbor not in visited:
                heapq.heappush(pq, (edge_weight, neighbor, current))

    return mst


# Check for negative weights
def has_negative_weights(graph):
    for u, v, data in graph.edges(data=True):
        if data.get('weight', 1) < 0:
            return True
    return False


import matplotlib.pyplot as plt
import networkx as nx


def visualize_graph(graph, highlight=None, path=None, mst=None, floyd_paths=None, title="Graph Visualization"):
    """
    Visualizes the graph and highlights specific features (shortest path, minimum spanning tree, or other elements).

    Parameters:
    - graph: NetworkX graph to be visualized
    - highlight: List of nodes to highlight
    - path: List of nodes representing the shortest path (highlighted in red)
    - mst: List of edges in the minimum spanning tree (highlighted in green)
    - floyd_paths: List of edges across all shortest paths (e.g., from Floyd-Warshall algorithm) highlighted in purple
    - title: Title of the graph visualization
    """
    # Retrieve node positions if defined in the graph
    pos = nx.get_node_attributes(graph, 'pos')
    if not pos:  # If positions are not specified, generate them automatically
        pos = nx.spring_layout(graph)

    # Get node labels (names of settlements or other attributes)
    labels = nx.get_node_attributes(graph, 'name')

    # Initialize the plot
    plt.figure(figsize=(12, 12))
    nx.draw(graph, pos, with_labels=False, node_size=50, node_color='blue', edge_color='gray')

    # Highlight the shortest path (if provided)
    if path:
        path_edges = [(path[i], path[i + 1]) for i in range(len(path) - 1)]
        nx.draw_networkx_edges(graph, pos, edgelist=path_edges, edge_color='red', width=2)
        nx.draw_networkx_nodes(graph, pos, nodelist=path, node_color='black', node_size=100)

    # Highlight the minimum spanning tree (if provided)
    if mst:
        nx.draw_networkx_edges(graph, pos, edgelist=mst, edge_color='green', width=2)

    # Highlight all shortest paths (if provided, e.g., from Floyd-Warshall algorithm)
    if floyd_paths:
        nx.draw_networkx_edges(graph, pos, edgelist=floyd_paths, edge_color='purple', width=2)

    # Highlight specific nodes (if provided)
    if highlight:
        nx.draw_networkx_nodes(graph, pos, nodelist=highlight, node_color='yellow', node_size=150)

    # Add labels to the nodes
    for node, (x, y) in pos.items():
        if node in labels and labels[node] is not None:
            plt.text(x, y, labels[node], fontsize=8, color='black', bbox=dict(facecolor='white', edgecolor='none'))

    # Set the title and display the plot
    plt.title(title)
    plt.show()


if __name__ == '__main__':
    # Pipeline
    use_length = True
    adj_list = graph_to_adj_list(graph, use_length_as_weight=use_length)
    # Mapování indexů na skutečné uzly
    index_to_node = {i: node for i, node in enumerate(adj_list.keys())}
    node_to_index = {node: i for i, node in index_to_node.items()}

    # Shortest path
    print("\nFind shortest path")
    start_node = resolve_node("Bezno")  # Starting Settlement name
    end_node = resolve_node("Kněžmost")  # End Settlement name



    if has_negative_weights(graph):
        print("Negative weights detected - Bellman-Ford used")
        path, length = bellman_ford(adj_list, start_node, end_node)
    else:
        print("Dijkstra used")
        path, length = dijkstra(adj_list, start_node, end_node)

    # Map path back to names
    path_names = [vertex_to_name.get(node, f"Node {node}") for node in path]
   # print(f"Nejkratší cesta mezi '{start_node}' a '{end_node}': {path_names} s délkou {int(length)} m")
    if use_length:
        print(f"Shortest path between '{path_names[0]}' and '{path_names[-1]}': {path_names}")
        print(f", length: {round(length/1000,2)} km")
    else:
        print(f"Quickest path between '{path_names[0]}' and '{path_names[-1]}': {path_names}")
        print(f"Transportation time: {int(length/60)} min")
visualize_graph(graph,highlight=path,path=path)

# Compute and visualize MST
# according to kruskal
mst_k = kruskal(graph)
visualize_graph(graph,mst=mst_k)
# according to Prim
mst_p = prim(graph)
visualize_graph(graph,mst=mst_p)