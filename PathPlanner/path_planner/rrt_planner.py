import numpy as np
import matplotlib.pyplot as plt
from path_planner.utils import ObstaclesGrid

class Node:
    def __init__(self, x, y, parent=None):
        """
        Represents a node in the RRT tree.
        
        Args:
            x (float): X-coordinate of the node.
            y (float): Y-coordinate of the node.
            parent (Node, optional): Parent node in the tree.
        """
        self.x = x
        self.y = y
        self.parent = parent  

class RRTPlanner:
    def __init__(self, start, goal, map_size, obstacles, max_iter=500, step_size=5):
        """
        Initializes the RRT planner.

        Args:
            start (tuple): (x, y) coordinates of the start position.
            goal (tuple): (x, y) coordinates of the goal position.
            map_size (tuple): (width, height) of the environment.
            obstacles (ObstaclesGrid): Object that stores obstacle information.
            max_iter (int): Maximum number of iterations for RRT.
            step_size (float): Step size for expanding the tree.
        """
        self.start = Node(start[0], start[1])
        self.goal = Node(goal[0], goal[1])
        self.map_size = map_size
        self.obstacles = obstacles
        self.max_iter = max_iter
        self.step_size = step_size
        self.tree = [self.start] 

    def plan(self):
        """
        Implements the RRT algorithm to find a path from start to goal.

        Returns:
            list: A list of (x, y) tuples representing the path from start to goal.
        """
        for i in range(self.max_iter):
            rand_node = self.sample_random_point()  
            nearest_node = self.find_nearest_node(rand_node)  
            new_node = self.steer(nearest_node, rand_node) 

            if new_node and not self.is_colliding(new_node, nearest_node): 
                self.tree.append(new_node)

                if self.reached_goal(new_node):  
                    return self.construct_path(new_node) 
        
        print("Path not found.")
        return None

    def sample_random_point(self):
        """
        Samples a random point in the map.
        
        Returns:
            Node: A randomly sampled node.
        """
        # Randomly sample within the width (map_size[0]) and height (map_size[1]) range of the map
        rand_x = np.random.uniform(0, self.map_size[0])
        rand_y = np.random.uniform(0, self.map_size[1])
        return Node(rand_x, rand_y)

    def find_nearest_node(self, rand_node):
        """
        Finds the nearest node in the tree to a given random node.

        Args:
            rand_node (Node): The randomly sampled node.

        Returns:
            Node: The nearest node in the tree.
        """
        distances = [(node.x - rand_node.x)**2 + (node.y - rand_node.y)**2 for node in self.tree]
        nearest_idx = np.argmin(distances)
        return self.tree[nearest_idx]

    def steer(self, nearest_node, rand_node):
        """
        Generates a new node by moving from the nearest node toward the random node.

        Args:
            nearest_node (Node): The nearest node in the tree.
            rand_node (Node): The randomly sampled node.

        Returns:
            Node: A new node in the direction of rand_node.
        """
        dx = rand_node.x - nearest_node.x
        dy = rand_node.y - nearest_node.y
        dist = np.hypot(dx, dy)
        
        # If the random point is closer than the step size, use the random point as the new node
        if dist < self.step_size:
            new_x = rand_node.x
            new_y = rand_node.y
        else:
            # Otherwise, calculate the angle and move only step_size distance
            theta = np.arctan2(dy, dx)
            new_x = nearest_node.x + self.step_size * np.cos(theta)
            new_y = nearest_node.y + self.step_size * np.sin(theta)
            
        return Node(new_x, new_y, parent=nearest_node)

    def is_colliding(self, new_node, nearest_node):
        """
        Checks if the path between nearest_node and new_node collides with an obstacle.

        Args:
            new_node (Node): The new node to check.
            nearest_node (Node): The nearest node in the tree.

        Returns:
            bool: True if there is a collision, False otherwise.
        """

        # Divide the line segment into multiple small steps for checking, with a step size of 1 unit
        dist = np.hypot(new_node.x - nearest_node.x, new_node.y - nearest_node.y)
        steps = max(int(dist / 0.5), 1) 
        margin = 2  # robot safety radius, force away from obstacles
        
        for i in range(steps + 1):
            x = nearest_node.x + i * (new_node.x - nearest_node.x) / steps
            y = nearest_node.y + i * (new_node.y - nearest_node.y) / steps
            ix, iy = int(x), int(y)
            
            # check if the point is within the margin range
            if not self.obstacles.is_point_valid((ix, iy)) \
               or not self.obstacles.is_point_valid((ix + margin, iy)) \
               or not self.obstacles.is_point_valid((ix - margin, iy)) \
               or not self.obstacles.is_point_valid((ix, iy + margin)) \
               or not self.obstacles.is_point_valid((ix, iy - margin)):
                return True
                
        return False
        

    def reached_goal(self, new_node):
        """
        Checks if the goal has been reached.

        Args:
            new_node (Node): The most recently added node.

        Returns:
            bool: True if goal is reached, False otherwise.
        """

        dist = np.hypot(new_node.x - self.goal.x, new_node.y - self.goal.y)
        # If the distance is less than or equal to the step size, we consider the goal reached
        return dist <= self.step_size

    def construct_path(self, end_node):
        """
        Constructs the final path by backtracking from the goal node to the start node.

        Args:
            end_node (Node): The node at the goal position.

        Returns:
            list: A list of (x, y) tuples representing the path from start to goal.
        """

        path = []
        curr = end_node
        #   Find father
        while curr is not None:
            path.append((curr.x, curr.y))
            curr = curr.parent
            
        path.reverse()
        
        # Ensure the goal is accurately added to the path
        if np.hypot(path[-1][0] - self.goal.x, path[-1][1] - self.goal.y) > 0.1:
            path.append((self.goal.x, self.goal.y))
            
        return path
