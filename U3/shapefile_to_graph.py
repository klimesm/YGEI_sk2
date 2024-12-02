import geopandas as gpd
from shapely.geometry import Point, LineString, MultiLineString
import networkx as nx
import pandas as pd
import pickle

# Načtení dat z GeoPackage
silnice_file = "shapefiles/silnice_gp.gpkg"
sidla_file = "shapefiles/sidla_gp.gpkg"

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


# Zpracování grafu
graph = nx.Graph()
settlements_info = {}

for _, settlement in settlements.iterrows():
    settlement_point = settlement.geometry
    nearest_point, nearest_line = find_nearest_point_on_line(settlement_point, roads.geometry)

    if nearest_line and nearest_point:
        line1, line2, assigned_vertex = split_line_or_assign_vertex(
            nearest_line, nearest_point, graph.nodes, settlements_info
        )
        if line1 and line2:
            new_road1 = gpd.GeoDataFrame({'geometry': [line1]}, crs=roads.crs)
            new_road2 = gpd.GeoDataFrame({'geometry': [line2]}, crs=roads.crs)
            roads = pd.concat([roads, new_road1, new_road2], ignore_index=True)
            roads = roads[~(roads.geometry == nearest_line)]

# Sloučit krajní body do jednoho vrcholu (když se tam setkává více linií)
unique_points = set()

for line in roads.geometry:
    if isinstance(line, MultiLineString):
        for sub_line in line.geoms:
            unique_points.add(Point(sub_line.coords[0]))
            unique_points.add(Point(sub_line.coords[-1]))
    elif isinstance(line, LineString):
        unique_points.add(Point(line.coords[0]))
        unique_points.add(Point(line.coords[-1]))

# Přidání vrcholů do grafu
point_to_vertex = {point: i for i, point in enumerate(unique_points)}
for point in unique_points:
    graph.add_node(point_to_vertex[point], pos=(point.x, point.y),
                   is_settlement=settlements_info.get((point.x, point.y), False))

# Přidání nákladů
for _, road in roads.iterrows():
    if isinstance(road.geometry, MultiLineString):
        for sub_line in road.geometry.geoms:
            start_point = Point(sub_line.coords[0])  # První bod sub-linie
            end_point = Point(sub_line.coords[-1])  # Poslední bod sub-linie
            length = sub_line.length  # Délka sub-linie
            road_class = road["TRIDA"]
            speed = rychlostni_mapa.get(road_class, 50)  # Výchozí rychlost, pokud není uvedena
            cost = length / speed
            graph.add_edge(
                point_to_vertex[start_point],
                point_to_vertex[end_point],
                weight=cost,
                length=length
            )
    elif isinstance(road.geometry, LineString):
        start_point = Point(road.geometry.coords[0])
        end_point = Point(road.geometry.coords[-1])
        length = road.geometry.length
        road_class = road["TRIDA"]
        speed = rychlostni_mapa.get(road_class, 50)
        cost = length / speed
        graph.add_edge(
            point_to_vertex[start_point],
            point_to_vertex[end_point],
            weight=cost,
            length=length
        )
    else:
        raise TypeError(f"Neočekávaný typ geometrie: {type(road.geometry)}")


# Výstup (okres MB)
print(f"Počet vrcholů: {graph.number_of_nodes()}")
print(f"Počet hran: {graph.number_of_edges()}")

# Uložení grafu
with open("graph.pkl", "wb") as f:
    pickle.dump(graph, f)
