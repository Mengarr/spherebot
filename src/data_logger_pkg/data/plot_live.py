import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import time

# Path to the CSV file
file_path = 'compensated_heading.csv'

# Set up the plot
plt.style.use('seaborn')  # Optional: customize the plot style
fig, ax = plt.subplots()
ax.set_title("Live Data Plot")
ax.set_xlabel("Elapsed Time")
ax.set_ylabel("Data")

# Initialize the line to update
line, = ax.plot([], [], lw=2)

# Function to initialize the plot limits and line data
def init():
    # ax.set_xlim(0, 10)  # Adjust based on expected range of `elapsed_time`
    ax.set_ylim(0, 360)  # Adjust based on expected range of `data`
    line.set_data([], [])
    return line,

# Update function for animation
def update(frame):
    # Read the CSV file
    data = pd.read_csv(file_path)
    
    # Extract elapsed_time and data columns
    x_data = data['elapsed_time']
    y_data = data['data']
    
    # Update the plot limits dynamically based on new data range
    ax.set_xlim(min(x_data), max(x_data))
    ax.set_ylim(min(y_data), max(y_data))
    
    # Update the line with new data
    line.set_data(x_data, y_data)
    return line,

# Create the animation
ani = FuncAnimation(fig, update, init_func=init, blit=True, interval=1000)

# Show the plot
plt.show()
