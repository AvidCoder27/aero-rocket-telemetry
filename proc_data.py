import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import cumulative_trapezoid
from scipy.spatial.transform import Rotation as R

# Constants for sensor offsets
# These values are determined emprically
# There are just average  from sitting the IMU still
# Except for the accel_z offset, which is the average minus 9.81
GYRO_X_OFFSET = -0.01
GYRO_Y_OFFSET = 0.03
GYRO_Z_OFFSET = 0.01
ACCEL_X_OFFSET = 0.41
ACCEL_Y_OFFSET = -0.30
ACCEL_Z_OFFSET = 0.15

def load_data(filename):
    df = pd.read_csv(filename, header=None)
    df.columns = ['timestamp', 'accel_x', 'accel_y', 'accel_z', 'gyro_x', 'gyro_y', 'gyro_z']
    df['timestamp'] = df['timestamp'] * 1e-3  # Convert ms to seconds
    return df

def preprocess(df):
    df = df.sort_values(by='timestamp')
    df['timestamp'] = df['timestamp'] - df['timestamp'].iloc[0]  # start at 0
    df['dt'] = df['timestamp'].diff().fillna(0)
    return df

def integrate_motion(df):
    acc_body = df[['accel_x', 'accel_y', 'accel_z']].values
    gyro = df[['gyro_x', 'gyro_y', 'gyro_z']].values
    dt = df['dt'].values

    # Apply offsets to accelerometer and gyroscope data
    acc_body[:, 0] -= ACCEL_X_OFFSET
    acc_body[:, 1] -= ACCEL_Y_OFFSET
    acc_body[:, 2] -= ACCEL_Z_OFFSET
    gyro[:, 0] -= GYRO_X_OFFSET
    gyro[:, 1] -= GYRO_Y_OFFSET
    gyro[:, 2] -= GYRO_Z_OFFSET

    orientations = [R.identity()]
    for i in range(1, len(df)):
        delta_angle = gyro[i] * dt[i]
        r = R.from_rotvec(delta_angle)
        orientations.append(orientations[-1] * r)


    # Optional: apply a simple smoothing filter (rolling mean)
    #acc_body = pd.DataFrame(acc_body).rolling(window=5, min_periods=1).mean().values

    # Rotate into world frame
    #acc_world = np.array([orient.apply(acc) for orient, acc in zip(orientations, acc_body)])
    acc_world = acc_body

    # Subtract gravity from world frame Z
    acc_world[:, 2] -= 9.81

    # Integrate acceleration to velocity and position
    velocity = cumulative_trapezoid(acc_world, df['timestamp'], initial=0, axis=0)
    position = cumulative_trapezoid(velocity, df['timestamp'], initial=0, axis=0)

    return position, velocity, acc_world, acc_body


def plot_flight_path(position):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Full path
    ax.plot(position[:, 0], position[:, 1], position[:, 2], label='Flight Path', color='blue')

    # Start and end markers
    ax.scatter(position[0, 0], position[0, 1], position[0, 2], color='green', s=50, label='Launch (Start)')
    ax.scatter(position[-1, 0], position[-1, 1], position[-1, 2], color='red', s=50, label='Landing (End)')

    # Optional text labels
    ax.text(position[0, 0], position[0, 1], position[0, 2], 'Start', color='green')
    ax.text(position[-1, 0], position[-1, 1], position[-1, 2], 'End', color='red')

    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title('Rocket Flight Path')
    ax.legend()
    plt.show()


def main():
    filename = 'test_data/log_4_hold_flat.csv'  # Change this to your CSV filename
    df = load_data(filename)
    df = preprocess(df)
    position, velocity, acc_world, acc_body = integrate_motion(df)

    # Debug plot of Z-axis world-frame data
    plt.figure()
    plt.plot(df['timestamp'], acc_world[:, 2], label='Z Acc (world)')
    plt.plot(df['timestamp'], acc_body[:, 2], label='Z Acc (body)')
    plt.plot(df['timestamp'], velocity[:, 2], label='Z Vel')
    plt.plot(df['timestamp'], position[:, 2], label='Z Pos')
    plt.xlabel("Time (s)")
    plt.ylabel("Value")
    plt.title("Z-axis Acceleration, Velocity, and Position")
    plt.legend()
    plt.grid(True)
    plt.show()

    plot_flight_path(position)

if __name__ == '__main__':
    main()
