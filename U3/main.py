from math import *
from queue import *
#Graph 1 - Dijkstra
G = {
    1 : {2:8, 3:4, 5:2},
    2 : {1:8, 3:5, 4:2, 7:6, 8:7},
    3 : {1:4, 2:5, 6:3, 7:4},
    4 : {2:2, 9:3},
    5 : {1:2, 6:5},
    6 : {3:3, 5:5, 7:5, 8:7, 9:10},
    7 : {2:6, 3:4, 6:5, 8:3},
    8 : {2:7, 6:7, 7:3, 9:1},
    9 : {4:3, 6:10, 8:1}
}
# Graph 2 - Kruskal
# nodes
V = [1, 2, 3, 4, 5, 6, 7, 8, 9]
# edges
E = [
    [1, 2, 8],
    [1, 3, 4],
    [1, 5, 2],
    [2, 1, 8],
    [2, 3, 5],
    [2, 4, 2],
    [2, 7, 6],
    [2, 8, 7],
    [3, 1, 4],
    [3, 2, 5],
    [3, 6, 3],
    [3, 7, 4],
    [4, 2, 2],
    [4, 9, 3],
    [5, 1, 2],
    [5, 6, 5],
    [6, 3, 3],
    [6, 5, 5],
    [6, 7, 5],
    [6, 8, 7],
    [6, 9, 10],
    [7, 2, 6],
    [7, 3, 4],
    [7, 6, 5],
    [7, 8, 3],
    [8, 2, 7],
    [8, 6, 7],
    [8, 7, 3],
    [8, 9, 1],
    [9, 4, 3],
    [9, 6, 10],
    [9, 8, 1]
    ]
def rec_path(u,v,P):
    path = []
    # path shortening
    while v!=u and v !=-1:
        path.append(v)
        v = P[v]
    path.append(v)
    return path
def dijkstra(G,u,v):
    n = len(G)
    # No predecessors
    P = [-1]*(n+1)
    # Set infinite distances
    D = [inf]*(n+1)
    # create priority queue
    PQ = PriorityQueue()
    # starting node
    PQ.put((0,u))
    # starting node distance is 0
    D[u] = 0
    # until PQ is empty
    while not PQ.empty():
        # pop first node
        du,u = PQ.get()
        # iterate through neighbours
        for v,wuv in G[u].items():
            # edge relaxation
            # if better way found
            if D[v] >D[u] + wuv:
                # update distance
                D[v] = D[u]+wuv
                # update predecessor
                P[v] = u
                # add v to PQ
                PQ.put((D[v],v))
    # return predecessors and distance
    return P,D[v]
def find(u,P):
    # find parent node for u
    while P[u]!=u:
        u = P[u]
    return u
def union(u,v,P):
    # union-find
    # find roots
    root_u = find(u,P)
    root_v = find(v,P)
    # if in different subtrees
    if root_u != root_v:
        # connect u to v
        P[root_u] = root_v
def make_set(u,P):
    P[u] = u
def mst(V,E):
    # minimum spanning tree
    # tree
    T = []
    # init line weight
    wt = 0
    # number of nodes
    n = len(V)
    #
    P = [-1]*(n+1)
    # init trees
    for v in V:
        make_set(v,P)
    # sort edges
    ES = sorted(E, key=lambda it: it[2])
    # process sorted edges
    for e in ES:
        # take edge
        u, v, w = e
        # if roots u, v in different trees
        if (find(u, P) != find(v, P)):
            # union-find
            union(u, v, P)
            # add edge to tree
            T.append([u, v, w])
            # sum of weights
            wt = wt+w
    return T,wt


# Path using Dijkstra
P, dmin = dijkstra(G, 1, 9)
path = rec_path(1,9,P)
print(path,dmin)

# Boruvka/Kruskal
T, wt = mst(V,E)
print(T,wt)