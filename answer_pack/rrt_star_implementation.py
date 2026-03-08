
import cv2
import numpy as np

from path_planning import *
from path_planning.rrt_star_planner import RRTStarPlanner


class RRTStarImplementation(RRTStarPlanner):
    # TODO: implement your own version of preloop, step and postloop
    def get_nearest_node(self, rand_node:PathNode) -> PathNode:
        min_distance = float("inf")
        for node in self.visited_nodes:
            distance = calculate_node_distance(node, rand_node)
            if distance < min_distance:
                min_distance = distance
                nearest_node = node
        if min_distance == float("inf"):
            return None
        return nearest_node
    
    def steer(self, rand_node:PathNode, nearest_node:PathNode) -> PathNode:
        distance = calculate_node_distance(rand_node, nearest_node)
        if distance <= self.step_size:
            new_x = rand_node.coordinates.x
            new_y = rand_node.coordinates.y
        else:
            weight = self.step_size / distance
            new_x = nearest_node.coordinates.x + (rand_node.coordinates.x - nearest_node.coordinates.x) * weight
            new_y = nearest_node.coordinates.y + (rand_node.coordinates.y - nearest_node.coordinates.y) * weight
        new_node = PathNode(coordinates=PixelCoordinates(int(new_x), int(new_y)))
        if not check_collision_free(self.occupancy_map, nearest_node, new_node):
            return None
        new_node.parent = nearest_node
        new_node.cost = nearest_node.cost + calculate_node_distance(nearest_node, new_node)
        return new_node
    
    def get_near_nodes(self, new_node:PathNode, search_radius:float) -> list[PathNode]:
        near_nodes = []
        for node in self.visited_nodes:
            if calculate_node_distance(new_node, node) <= search_radius:
                if check_collision_free(self.occupancy_map, node, new_node):
                    near_nodes.append(node)
        return near_nodes
    
    def Reparent(self, new_node:PathNode, near_nodes:list[PathNode]) -> PathNode:
        for node in near_nodes:
            distance = node.cost + calculate_node_distance(node, new_node)
            if distance < new_node.cost:
                new_node.parent = node
                new_node.cost = distance
        return new_node
    
    def rewire(self, new_node:PathNode, near_nodes:list[PathNode]) -> None:
        for node in near_nodes:
            distance = new_node.cost + calculate_node_distance(new_node, node)
            if distance < node.cost:
                node.parent = new_node
                node.cost = distance
                

    def preloop(self):
        # This is for illustrative purposes only, feel free to modify
        self.visited_nodes:set[PathNode] = set()
        self.visited_nodes.add(self.start_node)

    def step(self):
        # ===== some given data/parameters =====
        self.start_node
        self.goal_node
        self.world_map # bgr
        self.occupancy_map # bool
        self.goal_threshold
        self.step_size # for steer
        self.search_radius # for reparent/rewire
        # ==========
        rand_node = self.sample_random_node()
        nearest_node = self.get_nearest_node(rand_node)
        new_node = self.steer(rand_node, nearest_node)
        if new_node is None:
            return
        near_nodes = self.get_near_nodes(new_node, self.search_radius)
        new_node = self.Reparent(new_node, near_nodes)
        self.rewire(new_node, near_nodes)
        self.visited_nodes.add(new_node)
        if calculate_node_distance(new_node, self.goal_node) <= self.goal_threshold:
            if check_collision_free(self.occupancy_map, new_node, self.goal_node):
                self.visited_nodes.add(self.goal_node)
                self.goal_node.parent = new_node
                self.goal_node.cost = new_node.cost + calculate_node_distance(new_node, self.goal_node)
                self.is_done.set()
                return
            

    def postloop(self):
        return (
            collect_path(self.goal_node), 
            self.visited_nodes
        )