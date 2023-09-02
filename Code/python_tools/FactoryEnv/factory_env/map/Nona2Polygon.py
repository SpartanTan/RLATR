import json
from shapely.geometry import Point, Polygon, MultiLineString
from shapely.ops import nearest_points

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as mplPolygon

np.seterr(all='ignore')
np.set_printoptions(precision=3, suppress=True)

class Nona2Polygon():
    """Non-Accessible Areas"""
    def __init__(self,
                 map_dir: str = "./config/nona_description.json",
                 safe_distance_to_wall = 0.6
                 ):
        # Create an empty grid map
        self.map_dir = map_dir

        with open(self.map_dir, "r") as file:
            # Load the JSON content into a Python dictionary
            data = json.load(file)

        self.polygons = [Polygon(coords) for coords in data["polygons"]]
        # Combine the exterior boundaries of all polygons into a MultiLineString
        self.boundary = MultiLineString([polygon.exterior for polygon in self.polygons])
        self.safe_distance_to_wall = safe_distance_to_wall
        self.current_nearest_point = None
        self.danger = False
        
    def check_if_collision(self, query_point:np.ndarray):
        """
        Check if a point is whith the non-accessible area
        Return true if it is.
        
        ### Parameters
        - `query_point`: np.ndarray, the point to be checked, shape: (2,) e,g, np.array([2, 2])
        """
        point = Point(query_point[0], query_point[1])
        for polygon in self.polygons:
            if polygon.contains(point):
                return True
        return False
    
    def find_nearest_boundary_point(self, query_point: np.ndarray):
        point = Point(query_point[0], query_point[1])
        # Find the nearest points on the boundary to the query_point
        nearest_point = nearest_points(point, self.boundary)[1]
        distance = nearest_point.distance(point)
        if distance < self.safe_distance_to_wall:
            self.danger = True
        else:
            self.danger = False
        self.current_nearest_point = np.array([nearest_point.x, nearest_point.y])
        self.current_nearest_distance = round(distance, 4)
        return self.danger, self.current_nearest_point, self.current_nearest_distance
    
    def find_boundary(self):
        x_min, x_max, y_min, y_max = float('inf'), float('-inf'), float('inf'), float('-inf')
        for polygon in self.polygons:
            x, y = polygon.exterior.xy
            x_min = min(x_min, min(x))
            x_max = max(x_max, max(x))
            y_min = min(y_min, min(y))
            y_max = max(y_max, max(y))
        return x_min, x_max, y_min, y_max
    
    def render(self, ax):
        for polygon in self.polygons:
            x, y = polygon.exterior.xy
            ax.plot(x, y, 'k-') # change 'k-' to any other color, linestyle
            ax.fill(x, y, alpha=0.3) # change alpha to control the transparency
        if self.danger:
            ax.scatter(self.current_nearest_point[0], self.current_nearest_point[1], color='blue', marker='x', s=50)
            ax.text(self.current_nearest_point[0]+0.2, self.current_nearest_point[1], f'{self.current_nearest_distance}', fontsize=8)
            
if __name__ == "__main__":
    
    polygon_map = Nona2Polygon()
    query_point = np.array([8, 3.5])
    danger, closest_point, distance = polygon_map.find_nearest_boundary_point(query_point)
    fig, ax = plt.subplots()
    plt.scatter(query_point[0], query_point[1], color='red', marker='o', s=50)
    polygon_map.render(ax)
    plt.show()
    
    