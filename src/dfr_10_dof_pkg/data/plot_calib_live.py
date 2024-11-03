import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import os

# Path to the data file
file_path = 'mag_out.txt'

# Set up the figure and axis
fig, ax = plt.subplots(figsize=(8, 8))
# ax.set_title('Magnetometer Data Calibration')
ax.set_xlabel('Field Strength (µT)', fontsize=15)
ax.set_ylabel('Field Strength (µT)', fontsize=15)

# Initialize scatter plots for each pair of axes
scatter_xy, = ax.plot([], [], 'ro', markersize=2, label='X vs Y')
scatter_yz, = ax.plot([], [], 'go', markersize=2, label='Y vs Z')
scatter_zx, = ax.plot([], [], 'bo', markersize=2, label='Z vs X')

# Customize plot appearance
ax.legend(loc='upper right', fontsize=15)
ax.set_aspect('equal', adjustable='datalim')
ax.grid(which="both", linestyle="--", linewidth=0.8)
ax.minorticks_on()  # Enable minor ticks for finer grid control
plt.tight_layout()

# Lists to hold data for plotting
x_data, y_data, z_data = [], [], []

def update_data():
    """Reads the latest data from the file."""
    if not os.path.exists(file_path):
        return
    
    with open(file_path, 'r') as f:
        lines = f.readlines()
    
    # Parse each line, expecting x,y,z format
    new_x, new_y, new_z = [], [], []
    for line in lines:
        try:
            x, y, z = map(float, line.strip().split(','))
            new_x.append(x)
            new_y.append(y)
            new_z.append(z)
        except ValueError:
            # Skip lines that cannot be parsed
            continue
    
    return new_x, new_y, new_z

def animate(frame):
    """Animation function to update the scatter plots."""
    global x_data, y_data, z_data
    
    new_x, new_y, new_z = update_data()
    if new_x and new_y and new_z:
        x_data = new_x
        y_data = new_y
        z_data = new_z
    
    # Update scatter plot data
    scatter_xy.set_data(x_data, y_data)
    scatter_yz.set_data(y_data, z_data)
    scatter_zx.set_data(z_data, x_data)
    
    ax.relim()
    ax.autoscale_view()

# Create animation
ani = animation.FuncAnimation(fig, animate, interval=500)

plt.show()
