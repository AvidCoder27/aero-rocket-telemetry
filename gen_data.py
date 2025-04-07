import numpy as np
import pandas as pd

# Create a time vector (60 seconds, sampled at 40 Hz)
sample_rate = 40
max_time = 60 # seconds
time = np.arange(0, max_time, 1/sample_rate)

# Simulate accelerometer data (LAX, LAY, LAZ) and quaternion (QW, QX, QY, QZ)
LAX = np.zeros_like(time)  # No horizontal acceleration initially
LAY = np.zeros_like(time)  # No horizontal acceleration initially
LAZ = -9.81 * np.ones_like(time)  # Gravity included throughout (negative because gravity pulls downward)

# Add launch thrust and bumpy motion during burn phase (simulating rocket behavior)
launch_start = int(len(time) * 0)
burn_end = int(len(time) * 0.25)
ground_hit = int(len(time) * 52.38 / max_time)

LAZ[launch_start:burn_end] += 20 + 1 * np.random.randn(burn_end - launch_start)
LAX[launch_start:burn_end] = 0.5 * np.random.randn(burn_end - launch_start)
LAY[launch_start:burn_end] = 0.5 * np.random.randn(burn_end - launch_start)

# Post-burn coasting with small oscillations
rng = np.random.default_rng()
LAX[burn_end:ground_hit] += 0.1 * np.sin(2 * np.pi * 0.5 * time[burn_end:ground_hit] + rng.random())  # Light oscillations
LAY[burn_end:ground_hit] += 0.1 * np.sin(2 * np.pi * 0.5 * time[burn_end:ground_hit] + rng.random())  # Light oscillations
LAZ[burn_end:ground_hit] += 0.1 * np.sin(2 * np.pi * 0.5 * time[burn_end:ground_hit] + rng.random())  # Light oscillations

LAZ[ground_hit:ground_hit+10] = 999; # crash!
LAZ[ground_hit+10:] = 0; # end of crash

# Simulate quaternion for rotation (small rotation throughout flight)
angle = np.pi / 20 * time
QW = np.cos(angle / 2)
QX = np.zeros_like(time)
QY = np.zeros_like(time)
QZ = np.sin(angle / 2)

# Combine everything into a DataFrame
imu_data = pd.DataFrame({
    'time': time,
    'LAX': LAX,
    'LAY': LAY,
    'LAZ': LAZ,
    'QW': QW,
    'QX': QX,
    'QY': QY,
    'QZ': QZ
})

# Save to CSV
imu_data.to_csv('./sim_data.csv', index=False)

'./sim_data.csv'
