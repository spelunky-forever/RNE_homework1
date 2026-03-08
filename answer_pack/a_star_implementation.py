
import cv2
import numpy as np

from path_planning import *
from path_planning.a_star_planner import AStarPlanner

import heapq
import itertools


class AStarImplementation(AStarPlanner):
    # TODO: implement your own version of preloop, step and postloop
    def preloop(self):
        # This is for illustrative purposes only, feel free to modify
        self.queue:set[PathNode] = set()
        self.g:dict[PathNode, float] = {}
        self.h:dict[PathNode, float] = {}
        self.visited_nodes:set[PathNode] = set()
        self.queue.add(self.start_node)
        self.g[self.start_node] = 0
        self.h[self.start_node] = calculate_node_distance(
            self.start_node, 
            self.goal_node
        )
        self.counter:int = itertools.count()
        self.priority_queue:list[tuple[float, int, PathNode]] = []
        self.g[self.start_node] = 0
        self.h[self.start_node] = calculate_node_distance(self.start_node, self.goal_node)
        heapq.heappush(self.priority_queue, (self.g[self.start_node] + self.h[self.start_node], next(self.counter), self.start_node))

    def step(self):
        # ===== some given data/parameters =====
        self.start_node
        self.goal_node
        self.world_map # bgr
        self.occupancy_map # bool
        self.goal_threshold
        self.grid_size # for sampling neighbor nodes
        # ==========
        # to sample neighbor nodes, use 
        # self.get_neighbor_nodes(current_node)

        if not self.priority_queue:
            self.is_done.set()
            return

        self.current_node = heapq.heappop(self.priority_queue)[2]
        if self.current_node in self.visited_nodes:
            return

        self.visited_nodes.add(self.current_node)

        if calculate_node_distance(self.current_node, self.goal_node) <= self.goal_threshold:
            if check_collision_free(self.occupancy_map, self.current_node, self.goal_node):
                self.visited_nodes.add(self.goal_node)
                self.goal_node.parent = self.current_node
                self.goal_node.cost = self.g[self.current_node] + calculate_node_distance(self.current_node, self.goal_node)
                self.is_done.set()
                return
        
        self.neighbornood = self.get_neighbor_nodes(self.current_node)
        for neighbor in self.neighbornood:
            self.new_g = self.g[self.current_node] + calculate_node_distance(self.current_node, neighbor)
            if self.new_g < self.g.get(neighbor, float('inf')):
                self.g[neighbor] = self.new_g
                self.h[neighbor] = calculate_node_distance(neighbor, self.goal_node)
                neighbor.parent = self.current_node
                neighbor.cost = self.new_g
                heapq.heappush(self.priority_queue, (self.g[neighbor] + self.h[neighbor], next(self.counter), neighbor))

    def postloop(self):
        return (
            collect_path(self.goal_node), 
            self.visited_nodes
        )