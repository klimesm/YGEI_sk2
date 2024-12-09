import geopandas as gpd
from shapely.geometry import Point, LineString, MultiLineString
import networkx as nx
import pandas as pd
import pickle

# Načtení dat z GeoPackage
silnice_file = "silnice_gp.gpkg"
sidla_file = "sidla_gp.gpkg"

# Načtení silnic
roads = gpd.read_file(silnice_file)
print("Sloupce v silnicích:", roads.columns)

# Načtení sídel
settlements = gpd.read_file(sidla_file)
print("Sloupce v sídlech:", settlements.columns)

# Rychlostní mapa
rychlostni_mapa = {1: 130, 2: 110, 3: 90, 4: 70, 5: 50, 6: 30}

# Přiřazení sídla k bodu na linii
def find_nearest_point_on_line(point, lines):
    """Najde nejbližší bod na linii."""
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


# Přidá nový vrchol grafu (sídlo), pokud bylo sídlo přiřazeno krajnímu bodu, tak pouze ponechá krajní bod.
def split_line_or_assign_vertex(line, point, vertices, settlements_info):
    """Rozdělí linii nebo přiřadí vrcholu informaci o příslušnosti k sídlu."""
    if isinstance(line, MultiLineString):
        min_dist = float("inf")
        closest_line = None
        for sub_line in line.geoms:
            dist = sub_line.distance(point)
            if dist < min_dist:
                min_dist = dist
                closest_line = sub_line
        line = closest_line

    if isinstance(line, LineString):
        coords = list(line.coords)
        point_coords = (point.x, point.y)
        distances = [Point(coord).distance(point) for coord in coords]
        split_index = distances.index(min(distances))

        if len(coords[:split_index + 1]) >= 2 and len(coords[split_index:]) >= 2:
            part1_coords = coords[:split_index + 1]
            part2_coords = coords[split_index:]

            if point_coords not in part1_coords:
                part1_coords.append(point_coords)
            if point_coords not in part2_coords:
                part2_coords.insert(0, point_coords)

            line1 = LineString(part1_coords)
            line2 = LineString(part2_coords)
            return line1, line2, None
        else:
            nearest_vertex = coords[split_index]
            settlements_info[nearest_vertex] = True
            return None, None, nearest_vertex

    raise TypeError(f"Neočekávaný typ geometrie: {type(line)}")


# Křivost
def calculate_crookedness(line):
    length = line.length
    start_point = Point(line.coords[0])
    end_point = Point(line.coords[-1])
    euclidean_distance = ((start_point.x - end_point.x) ** 2 + (start_point.y - end_point.y) ** 2) ** 0.5
    return length / euclidean_distance if euclidean_distance > 0 else 1  # Avoid division by zero


def make_unique_name(name, existing_names):
    """
    Přidá číslování k názvu, aby byl jedinečný.
    """
    original_name = name
    counter = 1
    while name in existing_names:
        name = f"{original_name}_{counter}"
        counter += 1
    return name


# Zpracování grafu
graph = nx.Graph()
settlements_info = {}
# Add vertices to the graph with names for "sídlo" vertices
point_to_vertex = {}
vertex_to_name = {}  # Map vertices to their names (if applicable)

existing_names = set(vertex_to_name.values())
for _, settlement in settlements.iterrows():
    settlement_point = settlement.geometry
    nearest_point, nearest_line = find_nearest_point_on_line(settlement_point, roads.geometry)

    # Add the settlement point to the graph with its name
    if nearest_point:
        name = settlement["NAZEV"]
        name = make_unique_name(name, existing_names)
        existing_names.add(name)
        point = (nearest_point.x, nearest_point.y)
        if point not in point_to_vertex:
            vertex_id = len(point_to_vertex)
            point_to_vertex[point] = vertex_id
            vertex_to_name[vertex_id] = name
            graph.add_node(vertex_id, pos=(nearest_point.x, nearest_point.y), is_settlement=True, name=name)

# Process unique points for roads
for line in roads.geometry:
    if isinstance(line, MultiLineString):
        for sub_line in line.geoms:
            start_point = Point(sub_line.coords[0])
            end_point = Point(sub_line.coords[-1])
            for point in [start_point, end_point]:
                point_coords = (point.x, point.y)
                if point_coords not in point_to_vertex:
                    vertex_id = len(point_to_vertex)
                    point_to_vertex[point_coords] = vertex_id
                    graph.add_node(vertex_id, pos=(point.x, point.y), is_settlement=False, name=None)
    elif isinstance(line, LineString):
        start_point = Point(line.coords[0])
        end_point = Point(line.coords[-1])
        for point in [start_point, end_point]:
            point_coords = (point.x, point.y)
            if point_coords not in point_to_vertex:
                vertex_id = len(point_to_vertex)
                point_to_vertex[point_coords] = vertex_id
                graph.add_node(vertex_id, pos=(point.x, point.y), is_settlement=False, name=None)

# Update edges to use labeled vertices
for _, road in roads.iterrows():
    if isinstance(road.geometry, MultiLineString):
        for sub_line in road.geometry.geoms:
            start_point = Point(sub_line.coords[0])
            end_point = Point(sub_line.coords[-1])
            length = sub_line.length
            crookedness = calculate_crookedness(sub_line)
            road_class = road["TRIDA"]
            speed = rychlostni_mapa.get(road_class, 50)
            cost = (length * crookedness) / speed
            graph.add_edge(
                point_to_vertex[(start_point.x, start_point.y)],
                point_to_vertex[(end_point.x, end_point.y)],
                weight=cost,
                length=length,
                crookedness=crookedness
            )
            # Print edge information
            print(f"Edge added: Start=({start_point.x}, {start_point.y}), "
                  f"End=({end_point.x}, {end_point.y}), "
                  f"Length={length:.2f}, Crookedness={crookedness:.2f}, "
                  f"Speed={speed}, Weight={cost:.2f}")
    elif isinstance(road.geometry, LineString):
        start_point = Point(road.geometry.coords[0])
        end_point = Point(road.geometry.coords[-1])
        length = road.geometry.length
        crookedness = calculate_crookedness(road.geometry)
        road_class = road["TRIDA"]
        speed = rychlostni_mapa.get(road_class, 50)
        cost = (length * crookedness) / speed
        graph.add_edge(
            point_to_vertex[(start_point.x, start_point.y)],
            point_to_vertex[(end_point.x, end_point.y)],
            weight=cost,
            length=length,
            crookedness=crookedness
        )
        # Print edge information
        print(f"Edge added: Start=({start_point.x}, {start_point.y}), "
              f"End=({end_point.x}, {end_point.y}), "
              f"Length={length:.2f}, Crookedness={crookedness:.2f}, "
              f"Speed={speed}, Weight={cost:.2f}")
    else:
        raise TypeError(f"Unsupported geometry type: {type(road.geometry)}")


# Save the updated graph
with open("graph.pkl", "wb") as f:
    pickle.dump(graph, f)

print("All vertices:")
for node, data in graph.nodes(data=True):
    print(f"Vertex {node}: {data}")
