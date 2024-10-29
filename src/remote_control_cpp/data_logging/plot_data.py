import pandas as pd
import matplotlib.pyplot as plt

# Load the data
data = pd.read_csv("motor_speed_data.csv")

# Plot for Motor A
plt.figure(figsize=(10, 6))
plt.plot(data["Time"], data["RefMotorASpeed"], label="RefMotorASpeed")
plt.plot(data["Time"], data["MotorASpeed"], label="MotorASpeed")
plt.xlabel("Time", fontsize=14)
plt.ylabel("Speed", fontsize=14)
plt.title("Motor A Speed vs Time", fontsize=16)
plt.legend(fontsize=12)
plt.grid()

# Plot for Motor B
plt.figure(figsize=(10, 6))
plt.plot(data["Time"], data["RefMotorBSpeed"], label="RefMotorBSpeed")
plt.plot(data["Time"], data["MotorBSpeed"], label="MotorBSpeed")
plt.xlabel("Time", fontsize=14)
plt.ylabel("Speed", fontsize=14)
plt.title("Motor B Speed vs Time", fontsize=16)
plt.legend(fontsize=12)
plt.grid()

# Display the plots
plt.show()
