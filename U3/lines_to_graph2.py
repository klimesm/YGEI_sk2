from collections import *
from queue import *
from numpy import *
  
def loadEdges(file_name):
    #Convert list of lines to the graph
    PS = []
    PE = []
    W = []
    with open(file_name) as f:
        for line in f:
            #Split
            x1, y1, x2, y2, w = line.split()
            
            #Add start, end points and weights to the list
            PS.append((float(x1), float(y1)))
            PE.append((float(x2), float(y2)))
            W.append(float(w))
    return PS, PE, W

def pointsToIDs(P):
    #Create a map: key = coordinates, value = id
    D = {}
    for i in range(len(P)):
        D[(P[i][0], P[i][1])] = i
        
    return D

def edgesToGraph(D, PS, PE, W):
    #Convert edges to undirected graph
    G = defaultdict(dict)

    for i in range(len(PS)):
        G[D[PS[i]]][D[PE[i]]] = W[i]
        G[D[PE[i]]][D[PS[i]]] = W[i]

    print(G)
    return G

#Load edges
file = 'graph.txt'

PS, PE, W = loadEdges(file)

#Merge lists and remove unique points
PSE = PS + PE
PSE=unique(PSE,axis=0).tolist()
PSE.insert(0, [1000000, 1000000])

#Edges to graph
D = pointsToIDs(PSE)
G = edgesToGraph(D, PS, PE, W)

