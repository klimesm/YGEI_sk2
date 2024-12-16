import geopandas as gpd
from shapely.geometry import Point, LineString, MultiLineString
import networkx as nx
import pandas as pd
import pickle

# Load data from GeoPackage
silnice_file = "silnice_gp.gpkg"
sidla_file = "sidla_gp.gpkg"

# Load roads
roads = gpd.read_file(silnice_file)
print("Columns in roads:", roads.columns)

# Load settlements
settlements = gpd.read_file(sidla_file)
print("Columns in settlements:", settlements.columns)

# Speed map
rychlostni_mapa = {1: 130, 2: 110, 3: 90, 4: 70, 5: 50, 6: 30}

def calculate_crookedness(line):
    """Calculate the crookedness of a road line."""
    length = line.length
    start_point = Point(line.coords[0])
    end_point = Point(line.coords[-1])
    euclidean_distance = ((start_point.x - end_point.x) ** 2 + (start_point.y - end_point.y) ** 2) ** 0.5
    return length / euclidean_distance if euclidean_distance > 0 else 1  # Avoid division by zero

def add_edge_with_length(graph, u, v, weight, pos_u, pos_v):
    """Add an edge to the graph ensuring both weight and length attributes are present."""
    length = Point(pos_u).distance(Point(pos_v))  # Calculate Euclidean distance
    graph.add_edge(u, v, weight=weight, length=length)

def find_nearest_point_on_line(point, lines):
    """Find the nearest point on a line to a given point."""
    min_dist = float("inf")
    nearest_point = None
    nearest_line = None
    for line in lines:
        if isinstance(line, MultiLineString):
            for sub_line in line.geoms:
                projected_point = sub_line.interpolate(sub_line.project(point))
                dist = point.distance(projected_point)
                if dist < min_dist:
                    min_dist = dist
                    nearest_point = projected_point
                    nearest_line = sub_line
        elif isinstance(line, LineString):
            projected_point = line.interpolate(line.project(point))
            dist = point.distance(projected_point)
            if dist < min_dist:
                min_dist = dist
                nearest_point = projected_point
                nearest_line = line
    return nearest_point, nearest_line

# Initialize graph and mappings
graph = nx.Graph()
point_to_vertex = {}
vertex_to_name = {}

# Process settlements and roads
for _, settlement in settlements.iterrows():
    settlement_point = settlement.geometry
    nearest_point, nearest_line = find_nearest_point_on_line(settlement_point, roads.geometry)

    if nearest_line and nearest_point:
        coords = list(nearest_line.coords)
        point_coords = (nearest_point.x, nearest_point.y)
        distances = [Point(coord).distance(settlement_point) for coord in [coords[0], coords[-1]]]
        split_index = distances.index(min(distances))

        # Add settlement as a vertex
        name = settlement["NAZEV"]
        settlement_coords = (settlement_point.x, settlement_point.y)
        if settlement_coords not in point_to_vertex:
            vertex_id = len(point_to_vertex)
            point_to_vertex[settlement_coords] = vertex_id
            vertex_to_name[vertex_id] = name
            graph.add_node(vertex_id, pos=settlement_coords, is_settlement=True, name=name)

        # Connect the settlement to the nearest vertex on the road
        nearest_vertex_coords = coords[split_index]
        if nearest_vertex_coords not in point_to_vertex:
            nearest_vertex_id = len(point_to_vertex)
            point_to_vertex[nearest_vertex_coords] = nearest_vertex_id
            graph.add_node(nearest_vertex_id, pos=nearest_vertex_coords, is_settlement=False, name=None)
        else:
            nearest_vertex_id = point_to_vertex[nearest_vertex_coords]

        weight = nearest_point.distance(Point(nearest_vertex_coords))
        add_edge_with_length(graph, vertex_id, nearest_vertex_id, weight, settlement_coords, nearest_vertex_coords)

# Add road vertices and edges
for line in roads.geometry:
    if isinstance(line, MultiLineString):
        for sub_line in line.geoms:
            coords = list(sub_line.coords)
            for i in range(len(coords) - 1):
                pos_u, pos_v = coords[i], coords[i + 1]
                if pos_u not in point_to_vertex:
                    vertex_u = len(point_to_vertex)
                    point_to_vertex[pos_u] = vertex_u
                    graph.add_node(vertex_u, pos=pos_u, is_settlement=False, name=None)
                else:
                    vertex_u = point_to_vertex[pos_u]

                if pos_v not in point_to_vertex:
                    vertex_v = len(point_to_vertex)
                    point_to_vertex[pos_v] = vertex_v
                    graph.add_node(vertex_v, pos=pos_v, is_settlement=False, name=None)
                else:
                    vertex_v = point_to_vertex[pos_v]

                crookedness = calculate_crookedness(sub_line)
                length = LineString([pos_u, pos_v]).length
                road_class = roads.loc[roads.geometry == line, "TRIDA"].iloc[0]
                speed = rychlostni_mapa.get(road_class, 50)/3.6
                weight = (length * crookedness) / speed
                add_edge_with_length(graph, vertex_u, vertex_v, weight, pos_u, pos_v)
    elif isinstance(line, LineString):
        coords = list(line.coords)
        for i in range(len(coords) - 1):
            pos_u, pos_v = coords[i], coords[i + 1]
            if pos_u not in point_to_vertex:
                vertex_u = len(point_to_vertex)
                point_to_vertex[pos_u] = vertex_u
                graph.add_node(vertex_u, pos=pos_u, is_settlement=False, name=None)
            else:
                vertex_u = point_to_vertex[pos_u]

            if pos_v not in point_to_vertex:
                vertex_v = len(point_to_vertex)
                point_to_vertex[pos_v] = vertex_v
                graph.add_node(vertex_v, pos=pos_v, is_settlement=False, name=None)
            else:
                vertex_v = point_to_vertex[pos_v]

            crookedness = calculate_crookedness(line)
            length = LineString([pos_u, pos_v]).length
            road_class = roads.loc[roads.geometry == line, "TRIDA"].iloc[0]
            speed = rychlostni_mapa.get(road_class, 50)/3.6
            weight = (length * crookedness) / speed
            add_edge_with_length(graph, vertex_u, vertex_v, weight, pos_u, pos_v)


# Utility function to calculate crookedness
def calculate_crookedness(line):
    length = line.length
    start_point = Point(line.coords[0])
    end_point = Point(line.coords[-1])
    euclidean_distance = start_point.distance(end_point)
    return length / euclidean_distance if euclidean_distance > 0 else 1

# Function to ensure edges have both weight and length
def add_edge_with_length(graph, u, v, length, speed, crookedness):
    weight = (length * crookedness) / speed
    graph.add_edge(u, v, weight=weight, length=length, crookedness=crookedness)
    print(f"Edge added: {u} - {v}, Length={length:.2f}, Crookedness={crookedness:.2f}, Weight={weight:.2f}")

# Find the nearest point on a line
def find_nearest_point_on_line(point, lines):
    min_dist = float("inf")
    nearest_point = None
    nearest_line = None

    for line in lines:
        if isinstance(line, MultiLineString):
            for sub_line in line.geoms:
                projected_point = sub_line.interpolate(sub_line.project(point))
                dist = point.distance(projected_point)
                if dist < min_dist:
                    min_dist = dist
                    nearest_point = projected_point
                    nearest_line = sub_line
        elif isinstance(line, LineString):
            projected_point = line.interpolate(line.project(point))
            dist = point.distance(projected_point)
            if dist < min_dist:
                min_dist = dist
                nearest_point = projected_point
                nearest_line = line
    return nearest_point, nearest_line

# Graph initialization
graph = nx.Graph()
point_to_vertex = {}
vertex_to_name = {}

# Add settlements and connect them to the graph
for _, settlement in settlements.iterrows():
    settlement_point = settlement.geometry
    nearest_point, nearest_line = find_nearest_point_on_line(settlement_point, roads.geometry)

    if nearest_point and nearest_line:
        # Add settlement vertex
        settlement_coords = (settlement_point.x, settlement_point.y)
        if settlement_coords not in point_to_vertex:
            vertex_id = len(point_to_vertex)
            point_to_vertex[settlement_coords] = vertex_id
            vertex_to_name[vertex_id] = settlement["NAZEV"]
            graph.add_node(vertex_id, pos=settlement_coords, is_settlement=True, name=settlement["NAZEV"])

        # Find the closest vertex on the line
        coords = list(nearest_line.coords)
        start_vertex = coords[0]
        end_vertex = coords[-1]

        # Safeguard: Ensure intermediary vertices are added only if meaningful
        nearest_vertex_coords = min([start_vertex, end_vertex], key=lambda p: Point(p).distance(settlement_point))
        if nearest_vertex_coords not in point_to_vertex:
            vertex_id = len(point_to_vertex)
            point_to_vertex[nearest_vertex_coords] = vertex_id
            vertex_to_name[vertex_id] = None  # No name for intermediary vertices
            graph.add_node(vertex_id, pos=nearest_vertex_coords, is_settlement=False)  # Mark as non-settlement

        # Add an edge between the settlement and the nearest line vertex
        graph.add_edge(
            point_to_vertex[settlement_coords],
            point_to_vertex[nearest_vertex_coords],
            weight=settlement_point.distance(Point(nearest_vertex_coords)),
            length=settlement_point.distance(Point(nearest_vertex_coords)),
            crookedness=1.0
        )


# Add road vertices and edges
for line in roads.geometry:
    if isinstance(line, MultiLineString):
        for sub_line in line.geoms:
            start_point = Point(sub_line.coords[0])
            end_point = Point(sub_line.coords[-1])
            for point in [start_point, end_point]:
                point_coords = (point.x, point.y)
                if point_coords not in point_to_vertex:
                    point_to_vertex[point_coords] = len(point_to_vertex)
                    graph.add_node(point_to_vertex[point_coords], pos=point_coords, is_settlement=False, name=None)

            length = sub_line.length
            crookedness = calculate_crookedness(sub_line)
            speed = rychlostni_mapa.get(roads["TRIDA"].iloc[0], 50)/3.6  # Default speed if not found
            add_edge_with_length(
                graph,
                point_to_vertex[(start_point.x, start_point.y)],
                point_to_vertex[(end_point.x, end_point.y)],
                length,
                speed,
                crookedness
            )
    elif isinstance(line, LineString):
        start_point = Point(line.coords[0])
        end_point = Point(line.coords[-1])
        for point in [start_point, end_point]:
            point_coords = (point.x, point.y)
            if point_coords not in point_to_vertex:
                point_to_vertex[point_coords] = len(point_to_vertex)
                graph.add_node(point_to_vertex[point_coords], pos=point_coords, is_settlement=False, name=None)

        length = line.length
        crookedness = calculate_crookedness(line)
        speed = rychlostni_mapa.get(roads["TRIDA"].iloc[0], 50)/3.6
        add_edge_with_length(
            graph,
            point_to_vertex[(start_point.x, start_point.y)],
            point_to_vertex[(end_point.x, end_point.y)],
            length,
            speed,
            crookedness
        )

# Save the graph
with open("graph.pkl", "wb") as f:
    pickle.dump(graph, f)

# Print all vertices
print("All vertices:")
for node, data in graph.nodes(data=True):
    print(f"Vertex {node}: {data}")

# Print all edges
print("All edges:")
for u, v, data in graph.edges(data=True):
    print(f"Edge {u} - {v}: {data}")
