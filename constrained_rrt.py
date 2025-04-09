import numpy as np
import random
from scipy.spatial import KDTree
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from pytransform3d.rotations import quaternion_slerp, matrix_from_quaternion
from pytransform3d.transformations import plot_transform

class RRTStarConstrained:
    def __init__(self, start, goal, plant_height, safe_height, max_iterations=10000, step_size=0.05, radius=0.1):
        # Define parameters
        self.start =start#[0.63, -0.16, 0.40]  # Start point in task space (x, y, z)
        self.goal =goal# [0.57, -0.43, 0.26]   # Goal point in task space (x, y, z)
        self.plant_height = plant_height#0.48       # Maximum height of the plant
        self.safe_height = safe_height #0.25        # safe height limit 

        self.max_iterations = max_iterations
        self.step_size = step_size
        self.radius = radius  # Radius for rewiring
        self.tree = [start]  # Initialize RRT tree with start node
        self.parent = {tuple(start): None}
        self.cost = {tuple(start): 0}  # Initialize cost-to-come
        self.goal_region=0.05
    def is_within_bounds(self, point):
        """Check if a point is within task space bounds."""
        return self.safe_height <= point[2] <= self.plant_height

    def sample_task_space(self):
         while True:
            """Sample a random point within task space bounds."""
            x = random.uniform(0,1) #(0.3, 0.6)  # Modify as per your workspace dimensions
            y = random.uniform(-0.5,0.5)                                               
            z = random.uniform(self.safe_height, self.plant_height)
            #if np.sqrt((x-self.start[0])**2 + (y-self.start[1])**2 + z**2) <= self.plant_height:
            return np.array([x, y, z])
      
       

    def nearest_neighbor(self, point):
        """Find the nearest neighbor in the tree."""
        tree = KDTree(self.tree)
        _, idx = tree.query(point)
        return self.tree[idx]

    def neighbors_within_radius(self, point):
        """Find neighbors within a radius in the tree."""
        tree = KDTree(self.tree)
        indices = tree.query_ball_point(point, self.radius)
        return [self.tree[i] for i in indices]

    def steer(self, from_point, to_point):
        """Move from `from_point` toward `to_point` by step size."""
        direction = to_point - from_point
        distance = np.linalg.norm(direction)
        if distance < self.step_size:
            return to_point
        direction = direction / distance
        return from_point + direction * self.step_size

    def cost_to_come(self, point):
        """Return the cost to come to a point."""
        return self.cost.get(tuple(point), float('inf'))

    def plan(self):
        """Plan a path using RRT* with task space constraints."""
        for i in range(self.max_iterations):
            sampled_point = self.sample_task_space()
            nearest_point = self.nearest_neighbor(sampled_point)
            new_point = self.steer(nearest_point, sampled_point)

            if self.is_within_bounds(new_point):
                # Calculate cost to come
                cost_to_new_point = self.cost_to_come(nearest_point) + np.linalg.norm(new_point - nearest_point)
               
                # Find neighbors for rewiring
                neighbors = self.neighbors_within_radius(new_point)
                min_cost = cost_to_new_point
                best_parent = nearest_point

                # Check for better parents
                for neighbor in neighbors:
                    cost_via_neighbor = self.cost_to_come(neighbor) + np.linalg.norm(new_point - neighbor)
                    if cost_via_neighbor < min_cost:
                        min_cost = cost_via_neighbor
                        best_parent = neighbor

                # Add the new point to the tree
                self.tree.append(new_point)
                self.parent[tuple(new_point)] = tuple(best_parent)
                self.cost[tuple(new_point)] = min_cost

                # Rewire neighbors
                for neighbor in neighbors:
                    cost_via_new_point = self.cost_to_come(new_point) + np.linalg.norm(new_point - neighbor)
                    if cost_via_new_point < self.cost_to_come(neighbor):
                        self.parent[tuple(neighbor)] = tuple(new_point)
                        self.cost[tuple(neighbor)] = cost_via_new_point

                # Check if we've reached the goal
                if np.linalg.norm(new_point - self.goal) <= self.goal_region:
                    self.parent[tuple(self.goal)] = tuple(new_point)
                    self.cost[tuple(self.goal)] = self.cost_to_come(new_point) + np.linalg.norm(new_point - self.goal)
                    return self.reconstruct_path()
        return None

    def reconstruct_path(self):
        """Reconstruct the path from start to goal."""
        path = []
        current = tuple(self.goal)
        while current is not None:
            path.append(current)
            current = self.parent.get(current)
        return path[::-1]

    def visualize(self,path, q_start, q_end, interpolated_quaternions):
        """Visualize the RRT* and the resulting path."""
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.set_title("RRT* with Task Space Constraints and Interpolated Orientations",fontsize=14)
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")
        ax.set_zlim(0, 1)

        # Plot the tree
        for child, parent in self.parent.items():
            if parent is not None:
                child = np.array(child)
                parent = np.array(parent)
                ax.plot([child[0], parent[0]], [child[1], parent[1]], [child[2], parent[2]], color='gray', linewidth=0.3)

        # Plot the path
        #if path:
            #path = np.array(path)
        ax.plot(path[:, 0], path[:, 1], path[:, 2], color='red', linewidth=4, label='Planned Path')

        # Highlight start and goal
        ax.scatter(self.start[0], self.start[1], self.start[2], color='green', s=100, label='Start')
        ax.scatter(self.goal[0], self.goal[1], self.goal[2], color='blue', s=100, label='Goal')
        
       
        
        # Extract the x, y, z positions from the path
        
        x_positions = path[:, 0]
        y_positions = path[:, 1]
        z_positions = path[:, 2]


        # Plot the trajectory (path)
        ax.plot(x_positions, y_positions, z_positions)

        # Plot interpolated frames at different positions along the trajectory
        for i in range(len(interpolated_quaternions)):
            q_interp = interpolated_quaternions[i]  # Get the interpolated quaternion
            
            # Create a transformation matrix from the quaternion
            transform_matrix = np.eye(4)
            transform_matrix[:3, :3] = matrix_from_quaternion(q_interp)  # Assign rotation matrix
            transform_matrix[:3, 3] = [x_positions[i], y_positions[i], z_positions[i]]  # Set translation
            
            # Plot each interpolated frame at a different position along the trajectory
            plot_transform(ax, A2B=transform_matrix, s=0.08)


       
        ax.legend(fontsize=12)
        plt.tight_layout()
        plt.show()
        
        return None

    def slerp(self, q_start, q_end, num_steps):
        """Perform SLERP (Spherical Linear Interpolation) between two quaternions."""
        t_values = np.linspace(0, 1, num_steps)  # Interpolating over the number of steps
        interpolated_quaternions = []
        q_start=np.array(q_start)
        q_end=np.array(q_end)
        for t in t_values:
            q_interp = quaternion_slerp(q_start, q_end, t)  # Interpolated quaternion
            interpolated_quaternions.append(q_interp)
        return interpolated_quaternions


"""
# Define parameters
start = [0.66, 0.046, 0.52]  # Start point in task space (x, y, z)
goal = [0.46, -0.47, 0.36]   # Goal point in task space (x, y, z)
plant_height = 0.52      # Maximum height of the plant
safe_height = 0.25        # safe height of the plant

# Create RRT* planner
rrt_star = RRTStarConstrained(start, goal, plant_height, safe_height)

# Plan a path
path = rrt_star.plan()

q_start=[-0.55,0.57,-0.42,3.24] #0.42
q_end=[-0.23,0.83,-0.48,-1.87] #0.1


# Visualize results
if path:
    print("Path found!")
    print(path)#Define start and end quaternions
    
    path = np.array(path)
    num_steps=len(path)
    interpolated_quaternions = rrt_star.slerp(q_start, q_end, num_steps)
    print(interpolated_quaternions)

    #rrt_star.visualize(path, q_start, q_end,interpolated_quaternions )
    
else:
    print("No path found within the iteration limit.")
"""
