import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import KDTree
from path_planner.utils import ObstaclesGrid

class Node:
    def __init__(self, x, y):
        """
        Represents a node in the PRM roadmap.

        Args:
            x (float): X-coordinate of the node.
            y (float): Y-coordinate of the node.
        """
        self.x = x
        self.y = y

class PRMPlanner:
    def __init__(self, start, goal, map_size, obstacles, num_samples=200, k_neighbors=10, step_size=5):
        """
        Initializes the PRM planner.

        Args:
            start (tuple): (x, y) coordinates of the start position.
            goal (tuple): (x, y) coordinates of the goal position.
            map_size (tuple): (width, height) of the environment.
            obstacles (ObstaclesGrid): Object that stores obstacle information.
            num_samples (int): Number of random samples for roadmap construction.
            k_neighbors (int): Number of nearest neighbors to connect in the roadmap.
            step_size (float): Step size used for collision checking.
        """
        self.start = Node(start[0], start[1])
        self.goal = Node(goal[0], goal[1])
        self.map_size = map_size
        self.obstacles = obstacles
        self.num_samples = num_samples
        self.k_neighbors = k_neighbors
        self.step_size = step_size
        self.roadmap = [] 
        self.edges = {} 

    def construct_roadmap(self):
        """
        Constructs the probabilistic roadmap by sampling nodes and connecting them.

        Returns:
        None
        """

        self.roadmap.append(self.start)
        self.roadmap.append(self.goal)
        
        # 1. Sample points
        for _ in range(self.num_samples):
            self.roadmap.append(self.sample_free_point())
            
        # 2. Initialize edge dictionary
        for node in self.roadmap:
            self.edges[node] = []
            
        # 3. Connect nodes
        for node in self.roadmap:
            neighbors = self.find_k_nearest(node, self.k_neighbors)
            for neighbor in neighbors:
                if not self.is_colliding(node, neighbor):
                    self.edges[node].append(neighbor)

    def sample_free_point(self):
        """
        Samples a random collision-free point in the environment.

        Returns:
        Node: A randomly sampled node.
        """
        margin = 2  
        while True:
            rand_x = np.random.uniform(0, self.map_size[0])
            rand_y = np.random.uniform(0, self.map_size[1])
            ix, iy = int(rand_x), int(rand_y)
            
            if self.obstacles.is_point_valid((ix, iy)) \
               and self.obstacles.is_point_valid((ix + margin, iy)) \
               and self.obstacles.is_point_valid((ix - margin, iy)) \
               and self.obstacles.is_point_valid((ix, iy + margin)) \
               and self.obstacles.is_point_valid((ix, iy - margin)):
                return Node(rand_x, rand_y)

    def find_k_nearest(self, node, k):
        """
        Finds the k-nearest neighbors of a node in the roadmap.

        Args:
            node (Node): The node for which neighbors are searched.
            k (int): The number of nearest neighbors to find.

        Returns:
            list: A list of k-nearest neighbor nodes.
        """

        if len(self.roadmap) <= 1:
            return []
            
        coords = np.array([[n.x, n.y] for n in self.roadmap])
        tree = KDTree(coords)
        
        # Search for k+1 nodes, because node itself (if already in the graph) will also be counted as a neighbor with distance 0
        distances, indices = tree.query([node.x, node.y], k=min(k + 1, len(self.roadmap)))
        
        neighbors = []
        # Handle the case where indices may be a scalar
        for idx in np.atleast_1d(indices):
            if self.roadmap[idx] != node:
                neighbors.append(self.roadmap[idx])
            if len(neighbors) == k:
                break
                
        return neighbors

    def is_colliding(self, node1, node2):
        """
        Checks if the path between two nodes collides with an obstacle.

        Args:
            node1 (Node): The first node.
            node2 (Node): The second node.

        Returns:
            bool: True if there is a collision, False otherwise.
        """

        dist = np.hypot(node2.x - node1.x, node2.y - node1.y)
        steps = max(int(dist / 1.0), 1) 
        margin = 2  # fix: keep safe distance when connecting nodes
        
        for i in range(steps + 1):
            x = node1.x + i * (node2.x - node1.x) / steps
            y = node1.y + i * (node2.y - node1.y) / steps
            ix, iy = int(x), int(y)
            
            if not self.obstacles.is_point_valid((ix, iy)) \
               or not self.obstacles.is_point_valid((ix + margin, iy)) \
               or not self.obstacles.is_point_valid((ix - margin, iy)) \
               or not self.obstacles.is_point_valid((ix, iy + margin)) \
               or not self.obstacles.is_point_valid((ix, iy - margin)):
                return True
                
        return False

    def plan(self):
        """
        Plans a path from start to goal using the constructed roadmap.

        Returns:
        list: A list of (x, y) tuples representing the path.
        """

        self.construct_roadmap()
        
        from queue import PriorityQueue
        open_set = PriorityQueue()
        # Store format: (cost, node ID to avoid comparison conflicts, node object)
        open_set.put((0, id(self.start), self.start))
        
        came_from = {self.start: None}
        cost_so_far = {self.start: 0}
        
        while not open_set.empty():
            _, _, current = open_set.get()
            
            if current == self.goal:
                break
                
            for next_node in self.edges.get(current, []):
                new_cost = cost_so_far[current] + np.hypot(next_node.x - current.x, next_node.y - current.y)
                
                if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                    cost_so_far[next_node] = new_cost
                    # Add heuristic distance (A* logic)
                    priority = new_cost + np.hypot(self.goal.x - next_node.x, self.goal.y - next_node.y)
                    open_set.put((priority, id(next_node), next_node))
                    came_from[next_node] = current
                    
        if self.goal not in came_from:
            print("Path not found.")
            return None
            
        # Backtrack path
        path = []
        current = self.goal
        while current is not None:
            path.append((current.x, current.y))
            current = came_from.get(current)
            
        path.reverse()
        return path
