import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import pandas as pd
import numpy as np

# Function to create a 3D box
def create_box(center, half_size):
    x, y, z = center
    hx, hy, hz = half_size
    vertices = [
        [x - hx, y - hy, z - hz],
        [x + hx, y - hy, z - hz],
        [x + hx, y + hy, z - hz],
        [x - hx, y + hy, z - hz],
        [x - hx, y - hy, z + hz],
        [x + hx, y - hy, z + hz],
        [x + hx, y + hy, z + hz],
        [x - hx, y + hy, z + hz]
    ]
    faces = [
        [vertices[0], vertices[1], vertices[2], vertices[3]],
        [vertices[4], vertices[5], vertices[6], vertices[7]],
        [vertices[0], vertices[1], vertices[5], vertices[4]],
        [vertices[2], vertices[3], vertices[7], vertices[6]],
        [vertices[1], vertices[2], vertices[6], vertices[5]],
        [vertices[4], vertices[7], vertices[3], vertices[0]]
    ]
    return faces

# Read trajectory and obstacles
trajectory_df = pd.read_csv('../data/trajectory.txt', names=['t', 'x', 'y', 'z'])
obstacles_df = pd.read_csv('../data/obstacles.csv', names=['x', 'y', 'z', 'hx', 'hy', 'hz'])

# Determine the bounding box for the trajectory
xmin, ymin, zmin = trajectory_df[['x', 'y', 'z']].min()
xmax, ymax, zmax = trajectory_df[['x', 'y', 'z']].max()

# Add a margin to the bounding box
margin = 10
xmin, ymin, zmin = xmin - margin, ymin - margin, zmin - margin
xmax, ymax, zmax = xmax + margin, ymax + margin, zmax + margin

# Filter obstacles within the bounding box of the trajectory
filtered_obstacles = obstacles_df[
    (obstacles_df['x'] - obstacles_df['hx'] <= xmax) &
    (obstacles_df['x'] + obstacles_df['hx'] >= xmin) &
    (obstacles_df['y'] - obstacles_df['hy'] <= ymax) &
    (obstacles_df['y'] + obstacles_df['hy'] >= ymin) &
    (obstacles_df['z'] - obstacles_df['hz'] <= zmax) &
    (obstacles_df['z'] + obstacles_df['hz'] >= zmin)
]

# Plotting
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot trajectory
ax.plot(trajectory_df['x'], trajectory_df['y'], trajectory_df['z'], c='m', label='Trajectory')
ax.scatter(trajectory_df['x'], trajectory_df['y'], trajectory_df['z'], c='r', marker='o', s=10)  # Optional: scatter plot of waypoints

# Plot obstacles
for _, row in filtered_obstacles.iterrows():
    center = (row['x'], row['y'], row['z'])
    half_size = (row['hx'], row['hy'], row['hz'])
    faces = create_box(center, half_size)
    poly3d = Poly3DCollection(faces, alpha=0.5, edgecolors='r')
    ax.add_collection3d(poly3d)

# Set labels and title
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('3D Trajectory and Obstacles Visualization')

# Set limits
ax.set_xlim([xmin, xmax])
ax.set_ylim([ymin, ymax])
ax.set_zlim([zmin, zmax])

plt.legend()
plt.show()
