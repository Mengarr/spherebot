import pandas as pd
import matplotlib.pyplot as plt

# Variables for file settings
motor_state_file = 'motor_joint_vars_state.csv'
ref_state_file = 'motor_joint_vars.csv'

# Read the CSV files
motor_state = pd.read_csv(motor_state_file)
ref_state = pd.read_csv(ref_state_file)

# Calculate differences
displacement_diff = ref_state['u_ref (mm)'] - motor_state['u_meas (mm)']
angular_velocity_diff = ref_state['alphadot_ref (rad/s)'] - motor_state['alphadot_meas (rad/s)']
angle_diff = ref_state['phi_ref (rad)'] - motor_state['phi_meas (rad)']

# # Configure plot style
# plt.rcParams.update({
#     "font.family": "serif",
#     "font.serif": ["Times New Roman"],
#     "text.usetex": True,
#     "font.size": 14
# })

# Create a figure with 3 subplots
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(8, 12), sharex=True)

# Plot u_ref - u_meas
ax1.plot(motor_state['elapsed_time'], displacement_diff, label='Displacement Difference')
ax1.set_ylabel('Displacement (mm)', fontsize=20)
ax1.tick_params(axis='y', labelsize=20)
ax1.grid(which="both", linestyle="--", linewidth=0.8)
ax1.minorticks_on()

# Plot alphadot_ref - alphadot_meas
ax2.plot(motor_state['elapsed_time'], angular_velocity_diff, label='Angular Velocity Difference')
ax2.set_ylabel('Angular Displacement (rad)', fontsize=20)
ax2.tick_params(axis='y', labelsize=20)
ax2.grid(which="both", linestyle="--", linewidth=0.8)
ax2.minorticks_on()

# Plot phi_ref - phi_meas
ax3.plot(motor_state['elapsed_time'], angle_diff, label='Angle Difference')
ax3.set_xlabel('Time (s)', fontsize=20)
ax3.set_ylabel('Angular Displacement (rad)', fontsize=20)
ax3.tick_params(axis='y', labelsize=20)
ax3.grid(which="both", linestyle="--", linewidth=0.8)
ax3.minorticks_on()

# Customize the x-axis label for all subplots
ax3.tick_params(axis='x', labelsize=20)

# Show the plot
plt.tight_layout()
plt.show()
