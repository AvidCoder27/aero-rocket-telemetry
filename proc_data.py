import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from pandas import DataFrame
from scipy.integrate import cumulative_trapezoid
from scipy.spatial.transform import Rotation as R
from matplotlib.animation import FuncAnimation
from scipy.signal import butter, filtfilt
import requests
import re
from datetime import datetime
from zoneinfo import ZoneInfo

# Time Constants for grabbing API data
MONTH = 4
DAY = 18
HOUR = 20
MINUTE = 30

# Log file name:
LOG_DIR = "flight_data/"
LOG_FILE = "may_15_third_water.csv"

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

# Filter out frequencies above these values
ACCEL_HIGH_CUT = 7  # Hz
VEL_HIGH_CUT = 5  # Hz
POS_HIGH_CUT = 5  # Hz

# Filter out frequencies below these values
VEL_LOW_CUT = 0.05  # Hz
POS_LOW_CUT = 0.05  # Hz

SHOW_VERTICAL_LINES = True  # Set to True to show vertical lines at key times
DO_WORLD_FRAME_ROTATION = True  # Set to True to rotate the accelerometer data into the world frame
WINDOW_START = 208
WINDOW_END = 216
DISPLAY_DEBUG = True  # Set to True to display debug plots
SUBTRACT_GRAVITY = True  # Set to True to subtract gravity from the world-frame accelerometer data
FILTER_ACCEL = True  # Set to True to filter the accelerometer data

LANDING_TIME = 214.8 # seconds
FLIGHT_TIME = 5.3 # seconds, by video (approx.)
FLIP_TIME = 211 # seconds, from data
LAUNCH_TIME = LANDING_TIME - FLIGHT_TIME
HALF_TIME = LAUNCH_TIME + (FLIGHT_TIME / 2)

def load_data(filename):
    df = pd.read_csv(filename, header=None)
    df.columns = ['timestamp', 'accel_x', 'accel_y', 'accel_z', 'gyro_x', 'gyro_y', 'gyro_z', 'pressure', 'temperature']
    df['timestamp'] = df['timestamp'] * 1e-3  # Convert ms to seconds
    df['pressure'] = df['pressure'] * 100 # Convert hPa to Pa
    df['temperature'] = df['temperature'] + 273.15 # Convert Celsius to Kelvin
    df = df.sort_values(by='timestamp')
    df['timestamp'] = df['timestamp'] - df['timestamp'].iloc[0]  # Start time at 0
    df['dt'] = df['timestamp'].diff().fillna(0) # Calculate delta times
    
    # mask the raw data with the window
    time = df['timestamp']
    mask = (time > WINDOW_START) & (time < WINDOW_END)
    df = df[mask].reset_index(drop=True)

    return df

# removes high frequency noise
def lowpass_filter(data, fs, highcut, order=4):
    b, a = butter(order, highcut, btype='lowpass', fs=fs)
    return filtfilt(b, a, data)

# removes low frequency noise
def highpass_filter(data, fs, lowcut, order=4):
    b, a = butter(order, lowcut, btype='highpass', fs=fs)
    return filtfilt(b, a, data)

def bandpass_filter(data, fs, lowcut, highcut, order=4):
    b, a = butter(order, [lowcut, highcut], btype='bandpass', fs=fs)
    return filtfilt(b, a, data)

def integrate_motion(df):
    acc_body = df[['accel_x', 'accel_y', 'accel_z']].values
    gyro = df[['gyro_x', 'gyro_y', 'gyro_z']].values
    timestamp = df['timestamp']
    fs = 1 / df['dt'].mean() # Sampling frequency
    print(f"Sampling frequency: {fs:.2f} Hz")

    # Apply offsets to accelerometer and gyroscope data
    acc_body[:, 0] -= ACCEL_X_OFFSET
    acc_body[:, 1] -= ACCEL_Y_OFFSET
    acc_body[:, 2] -= ACCEL_Z_OFFSET
    gyro[:, 0] -= GYRO_X_OFFSET
    gyro[:, 1] -= GYRO_Y_OFFSET
    gyro[:, 2] -= GYRO_Z_OFFSET

    # Integrate angular velocity to get rotation vectors
    rot_vecs = cumulative_trapezoid(gyro, timestamp, initial=0, axis=0)
    # Convert rotation vectors to rotation objects
    orientations = [R.from_rotvec(rv) for rv in rot_vecs]

    # Filter accel with a simple butterworth hi-pass filter
    # 4th order, cutoff at 5 Hz; filter each dimension separately
    # No need to low pass filter the acceleration data since no integration has been done yet 
    filter_acc_x = lowpass_filter(acc_body[:, 0], fs, highcut=ACCEL_HIGH_CUT)
    filter_acc_y = lowpass_filter(acc_body[:, 1], fs, highcut=ACCEL_HIGH_CUT)
    filter_acc_z = lowpass_filter(acc_body[:, 2], fs, highcut=ACCEL_HIGH_CUT)
    # recombine the dims after filtering
    filter_acc = np.column_stack((filter_acc_x, filter_acc_y, filter_acc_z))
    
    # undo filtering if not using it
    if not FILTER_ACCEL:
        filter_acc = acc_body.copy()

    # Rotate the filtered accelerations into the world frame
    if DO_WORLD_FRAME_ROTATION:
        acc_world = np.array([orient.apply(acc) for orient, acc in zip(orientations, filter_acc)])
    else:
        acc_world = filter_acc.copy()
    # Subtract gravity from world frame X
    if SUBTRACT_GRAVITY:
        acc_world[:, 0] += 9.81
        

    # Integrate acceleration to velocity and position
    velocity = cumulative_trapezoid(acc_world, timestamp, initial=0, axis=0)

    # filter velocity to remove high frequency noise from erroneous accelerometer data and low frequency noise from integration drift
    filter_vel_x = bandpass_filter(velocity[:, 0], fs, lowcut=VEL_LOW_CUT, highcut=VEL_HIGH_CUT)
    filter_vel_y = bandpass_filter(velocity[:, 1], fs, lowcut=VEL_LOW_CUT, highcut=VEL_HIGH_CUT)
    filter_vel_z = bandpass_filter(velocity[:, 2], fs, lowcut=VEL_LOW_CUT, highcut=VEL_HIGH_CUT)

    # recombine the dims after filtering
    filter_vel = np.column_stack((filter_vel_x, filter_vel_y, filter_vel_z))

    # Use Zero Velocity Update (ZUPT) to remove drift
    # This is done by getting the velocity bias of the stationary rocket
    # vel_bias = filter_vel[0:20].mean(axis=0)
    # filter_vel -= vel_bias

    # Integrate the filtered velocity to get position
    position = cumulative_trapezoid(filter_vel, timestamp, initial=0, axis=0)

    # Filter the position data
    filter_pos_x = bandpass_filter(position[:, 0], fs, lowcut=POS_LOW_CUT, highcut=POS_HIGH_CUT)
    filter_pos_y = bandpass_filter(position[:, 1], fs, lowcut=POS_LOW_CUT, highcut=POS_HIGH_CUT)
    filter_pos_z = bandpass_filter(position[:, 2], fs, lowcut=POS_LOW_CUT, highcut=POS_HIGH_CUT)
    
    # recombine the dims after filtering
    filter_pos = np.column_stack((filter_pos_x, filter_pos_y, filter_pos_z))

    return acc_body, gyro, orientations, filter_acc, acc_world, velocity, filter_vel, position, filter_pos

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

    # Find the max range to set equal axis scales
    max_range = np.max([
        position[:, 0].max() - position[:, 0].min(),
        position[:, 1].max() - position[:, 1].min(), 
        position[:, 2].max() - position[:, 2].min()
    ])

    # Get the mid points for each axis
    mid_x = (position[:, 0].max() + position[:, 0].min()) * 0.5
    mid_y = (position[:, 1].max() + position[:, 1].min()) * 0.5
    mid_z = (position[:, 2].max() + position[:, 2].min()) * 0.5

    # Set equal axis ranges around mid points
    ax.set_xlim(mid_x - max_range/2, mid_x + max_range/2)
    ax.set_ylim(mid_y - max_range/2, mid_y + max_range/2)
    ax.set_zlim(mid_z - max_range/2, mid_z + max_range/2)

    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title('Rocket Flight Path')
    ax.legend()
    plt.show()

def animate_orientation(orientations, timestamps):
    fig = plt.figure(figsize=(8, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    # Set axis limits
    ax.set_xlim([-1, 1])
    ax.set_ylim([-1, 1])
    ax.set_zlim([-1, 1])
    
    # Initialize arrows for each axis
    arrow_length = 0.5
    quiver_x = ax.quiver(0, 0, 0, arrow_length, 0, 0, color='r', label='X')
    quiver_y = ax.quiver(0, 0, 0, 0, arrow_length, 0, color='g', label='Y')
    quiver_z = ax.quiver(0, 0, 0, 0, 0, arrow_length, color='b', label='Z')
    
    def update(frame):
        # Get rotated basis vectors
        rotation = orientations[frame]
        x_rotated = rotation.apply([arrow_length, 0, 0])
        y_rotated = rotation.apply([0, arrow_length, 0])
        z_rotated = rotation.apply([0, 0, arrow_length])
        
        # Update arrows
        quiver_x.set_segments([[[0, 0, 0], x_rotated]])
        quiver_y.set_segments([[[0, 0, 0], y_rotated]])
        quiver_z.set_segments([[[0, 0, 0], z_rotated]])
        
        return quiver_x, quiver_y, quiver_z
    
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('Rocket Orientation')
    
    # Calculate frame interval based on timestamps
    frame_intervals = np.diff(timestamps)
    avg_interval = np.mean(frame_intervals[frame_intervals > 0]) * 1000
    
    anim = FuncAnimation(
        fig, update,
        frames=len(orientations),
        interval=avg_interval,  # Use actual time intervals
        blit=True
    )
    
    return anim

def plot_acceleration_fft(df):
    # Calculate sampling frequency from mean time step
    dt = df['dt'].mean()
    
    # Get acceleration data
    acc_x = df['accel_x'].values - ACCEL_X_OFFSET
    acc_y = df['accel_y'].values - ACCEL_Y_OFFSET
    acc_z = df['accel_z'].values - ACCEL_Z_OFFSET
    
    # Calculate FFT
    freq = np.fft.fftfreq(len(acc_x), d=dt)
    fft_x = np.abs(np.fft.fft(acc_x))
    fft_y = np.abs(np.fft.fft(acc_y))
    fft_z = np.abs(np.fft.fft(acc_z))
    
    # Create subplots
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 10))
    
    # Plot FFTs
    ax1.plot(freq[1:len(freq)//2], fft_x[1:len(freq)//2])
    ax1.set_title('X-Axis Acceleration FFT')
    ax1.set_xlabel('Frequency (Hz)')
    ax1.set_ylabel('Magnitude')
    ax1.grid(True)
    
    ax2.plot(freq[1:len(freq)//2], fft_y[1:len(freq)//2])
    ax2.set_title('Y-Axis Acceleration FFT')
    ax2.set_xlabel('Frequency (Hz)')
    ax2.set_ylabel('Magnitude')
    ax2.grid(True)
    
    ax3.plot(freq[1:len(freq)//2], fft_z[1:len(freq)//2])
    ax3.set_title('Z-Axis Acceleration FFT')
    ax3.set_xlabel('Frequency (Hz)')
    ax3.set_ylabel('Magnitude')
    ax3.grid(True)
    
    plt.tight_layout()
    plt.show()

def main():
    filename = LOG_DIR + LOG_FILE
    df = load_data(filename)

    outdf = DataFrame()
    outdf["timestamp"] = df["timestamp"]
    outdf["dt"] = df["dt"]
    outdf["acc_x"] = df["accel_x"]
    outdf["acc_y"] = df["accel_y"]
    outdf["acc_z"] = df["accel_z"]
    outdf["gyro_x"] = df["gyro_x"]
    outdf["gyro_y"] = df["gyro_y"]
    outdf["gyro_z"] = df["gyro_z"]
    outdf.to_csv("Unity Simulation/unity_data.csv", index=False, header=False)

    print("===========Gyro Stuff===========")
    # print out filtering high and low cut off frequencies
    print(f"Accel High Cut: {ACCEL_HIGH_CUT} Hz")
    print(f"Velocity High Cut: {VEL_HIGH_CUT} Hz")
    print(f"Velocity Low Cut: {VEL_LOW_CUT} Hz")
    print(f"Position High Cut: {POS_HIGH_CUT} Hz")
    print(f"Position Low Cut: {POS_LOW_CUT} Hz")

    # plot_acceleration_fft(df)
    acc_raw, gyro, orientations, acc_filter, acc_world, velocity, filter_vel, position, filter_pos = integrate_motion(df)
    euler_angles = [orient.as_euler('xyz', degrees=False) for orient in orientations]  # Returns angles in degrees
    euler_angles = np.array(euler_angles)

    if DISPLAY_DEBUG:
        x_acc_raw = acc_raw[:, 0]
        y_acc_raw = acc_raw[:, 1]
        z_acc_raw = acc_raw[:, 2]
        x_acc_filtered = acc_filter[:, 0]
        y_acc_filtered = acc_filter[:, 1]
        z_acc_filtered = acc_filter[:, 2]
        x_acc_world = acc_world[:, 0]
        y_acc_world = acc_world[:, 1]
        z_acc_world = acc_world[:, 2]
        x_vel = velocity[:, 0]
        y_vel = velocity[:, 1]
        z_vel = velocity[:, 2]
        x_vel_filt = filter_vel[:, 0]
        y_vel_filt = filter_vel[:, 1]
        z_vel_filt = filter_vel[:, 2]
        x_pos = position[:, 0]
        y_pos = position[:, 1]
        z_pos = position[:, 2]
        x_pos_filt = filter_pos[:, 0]
        y_pos_filt = filter_pos[:, 1]
        z_pos_filt = filter_pos[:, 2]
        time = df['timestamp']

        # Combine the raw acceleration data into a single vector using the Pythagorean theorem
        acc_magnitude = np.sqrt(x_acc_raw**2 + y_acc_raw**2 + z_acc_raw**2)
        acc_world_magnitude = np.sqrt(x_acc_world**2 + y_acc_world**2 + z_acc_world**2)
        gyro_magnitude = np.sqrt(gyro[:, 0]**2 + gyro[:, 1]**2 + gyro[:, 2]**2)

        plt.figure()
        # plt.plot(time, acc_magnitude, label='Acceleration Magnitude')
        # plt.plot(time, acc_world_magnitude, label='Acceleration World Magnitude')
        # plt.plot(time, gyro_magnitude, label='Gyro Magnitude')

        # plt.plot(time, x_acc_raw, label='X Accel Raw')
        # plt.plot(time, y_acc_raw, label='Y Accel Raw')
        # plt.plot(time, z_acc_raw, label='Z Accel Raw')

        # plt.plot(time, x_acc_filtered, label='X Accel Filtered')
        # plt.plot(time, y_acc_filtered, label='Y Accel Filtered')
        # plt.plot(time, z_acc_filtered, label='Z Accel Filtered')

        plt.plot(time, x_acc_world, label='X Accel World')
        # plt.plot(time, y_acc_world, label='Y Accel World')
        # plt.plot(time, z_acc_world, label='Z Accel World')

        # plt.plot(time, x_vel, label='X Vel')
        # plt.plot(time, y_vel, label='Y Vel')
        # plt.plot(time, z_vel, label='Z Vel')

        plt.plot(time, x_vel_filt, label='X Vel Filtered')
        # plt.plot(time, y_vel_filt, label='Y Vel Filtered')
        # plt.plot(time, z_vel_filt, label='Z Vel Filtered')

        plt.plot(time, x_pos, label='X Pos')
        # plt.plot(time, y_pos, label='Y Pos')
        # plt.plot(time, z_pos, label='Z Pos')

        # plt.plot(time, x_pos_filt, label='X Pos Filtered')
        # plt.plot(time, y_pos_filt, label='Y Pos Filtered')
        # plt.plot(time, z_pos_filt, label='Z Pos Filtered')

        # plt.plot(time, euler_angles[:, 0], label='Euler X')
        # plt.plot(time, euler_angles[:, 1], label='Euler Y')
        # plt.plot(time, euler_angles[:, 2], label='Euler Z')

        # plt.plot(time, gyro[:, 0], label='Gyro X')
        # plt.plot(time, gyro[:, 1], label='Gyro Y')
        # plt.plot(time, gyro[:, 2], label='Gyro Z')

        if SHOW_VERTICAL_LINES:
            # plot a vertical line at the landing time
            plt.axvline(x=LANDING_TIME, color='red', linestyle='--', label='Landing Time')
            # plot a vertical line at the launch time
            plt.axvline(x=LAUNCH_TIME, color='green', linestyle='--', label='Launch Time')
            # plot a vertical line at the flip time
            plt.axvline(x=FLIP_TIME, color='orange', linestyle='--', label='Flip Time')
            # plot a vertical line at the half time
            # plt.axvline(x=HALF_TIME, color='purple', linestyle='--', label='Half Time')
    
        # plot a dot at every data point for the raw acceleration data
        # plt.scatter(time, x_acc_raw, s=1, color='blue', alpha=0.5, label='X Accel Raw')

        plt.xlabel("Time (s)")
        plt.ylabel("Value")
        plt.title("Debugging Data")
        plt.legend()
        plt.grid(True)
        plt.show()

    # anim = animate_orientation(orientations, df['timestamp'])
    # anim.save("animation.mp4")
    # plot_flight_path(filter_pos)

if __name__ == '__main__':
    main()
