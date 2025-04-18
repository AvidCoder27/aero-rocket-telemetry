import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import matplotlib
from scipy.integrate import cumulative_trapezoid
from scipy.spatial.transform import Rotation as R
from matplotlib.animation import FuncAnimation
from scipy.signal import butter, filtfilt

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

ACCEL_FILTER_CUTOFF = 5  # Hz
VEL_FILTER_CUTOFF = 3  # Hz
POS_FILTER_CUTOFF = 2  # Hz

DO_WORLD_FRAME_ROTATION = True  # Set to True to rotate the accelerometer data into the world frame

# Log file name:
LOG_DIR = "test_data/"
LOG_FILE = "log_5_spin_jump.csv"

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

def butter_lowpass_filter(data, fs, cutoff, order=4):
    b, a = butter(order, cutoff, btype='lowpass', fs=fs)
    return filtfilt(b, a, data)

def integrate_motion(df):
    acc_body = df[['accel_x', 'accel_y', 'accel_z']].values
    gyro = df[['gyro_x', 'gyro_y', 'gyro_z']].values
    dt = df['dt'].values
    fs = 1 / dt.mean()  # Sampling frequency
    print(f"Sampling frequency: {fs:.2f} Hz")

    # Apply offsets to accelerometer and gyroscope data
    acc_body[:, 0] -= ACCEL_X_OFFSET
    acc_body[:, 1] -= ACCEL_Y_OFFSET
    acc_body[:, 2] -= ACCEL_Z_OFFSET
    gyro[:, 0] -= GYRO_X_OFFSET
    gyro[:, 1] -= GYRO_Y_OFFSET
    gyro[:, 2] -= GYRO_Z_OFFSET

    # Integrate angular velocity to get rotation vectors
    rot_vecs = cumulative_trapezoid(gyro, df['timestamp'], initial=0, axis=0)
    # Convert rotation vectors to rotation objects
    orientations = [R.from_rotvec(rv) for rv in rot_vecs]

    # Filter accel with a simple butterworth hi-pass filter
    # 4th order, cutoff at 5 Hz; filter each dimension separately 
    filter_acc_x = butter_lowpass_filter(acc_body[:, 0], fs, cutoff=ACCEL_FILTER_CUTOFF)
    filter_acc_y = butter_lowpass_filter(acc_body[:, 1], fs, cutoff=ACCEL_FILTER_CUTOFF)
    filter_acc_z = butter_lowpass_filter(acc_body[:, 2], fs, cutoff=ACCEL_FILTER_CUTOFF)
    # recombine the dims after filtering
    filter_acc = np.column_stack((filter_acc_x, filter_acc_y, filter_acc_z))

    # Rotate the filtered accelerations into the world frame
    if (DO_WORLD_FRAME_ROTATION):
        acc_world = np.array([orient.apply(acc) for orient, acc in zip(orientations, filter_acc)])
    else:
        acc_world = filter_acc.copy()
    # Subtract gravity from world frame Z
    acc_world[:, 2] -= 9.81

    # Integrate acceleration to velocity and position
    velocity = cumulative_trapezoid(acc_world, df['timestamp'], initial=0, axis=0)

    # Filter the velocity with another butterworth filter but at a lower cutoff frequency 
    filter_vel_x = butter_lowpass_filter(velocity[:, 0], fs, cutoff=VEL_FILTER_CUTOFF)
    filter_vel_y = butter_lowpass_filter(velocity[:, 1], fs, cutoff=VEL_FILTER_CUTOFF)
    filter_vel_z = butter_lowpass_filter(velocity[:, 2], fs, cutoff=VEL_FILTER_CUTOFF)
    # recombine the dims after filtering
    filter_vel = np.column_stack((filter_vel_x, filter_vel_y, filter_vel_z))

    # Use Zero Velocity Update (ZUPT) to remove drift
    # This is done by getting the velocity bias of the stationary rocket
    vel_bias = filter_vel[0:20].mean(axis=0)
    filter_vel -= vel_bias

    # Integrate the filtered velocity to get position
    position = cumulative_trapezoid(filter_vel, df['timestamp'], initial=0, axis=0)

    # Filter the position with another butterworth filter but at a lower cutoff frequency
    filter_pos_x = butter_lowpass_filter(position[:, 0], fs, cutoff=POS_FILTER_CUTOFF)
    filter_pos_y = butter_lowpass_filter(position[:, 1], fs, cutoff=POS_FILTER_CUTOFF)
    filter_pos_z = butter_lowpass_filter(position[:, 2], fs, cutoff=POS_FILTER_CUTOFF)
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
    df = preprocess(df)
    # plot_acceleration_fft(df)
    acc_body, gyro, orientations, filter_acc, acc_world, velocity, filter_vel, position, filter_pos = integrate_motion(df)

    # Debug plot
    # truncate acc_body and filter_acc to only be from time 5 to 15 seconds
    # acc_body = acc_body[df['timestamp'] < 15]
    # filter_acc = filter_acc[df['timestamp'] < 15]
    # df = df[df['timestamp'] < 15]
    # acc_body = acc_body[df['timestamp'] > 5]
    # filter_acc = filter_acc[df['timestamp'] > 5]
    # df = df[df['timestamp'] > 5]

    plt.figure()
    #plt.plot(df['timestamp'], acc_body[:, 0], label='X Acc (body)')
    #plt.plot(df['timestamp'], acc_body[:, 1], label='Y Acc (body)')
    plt.plot(df['timestamp'], acc_body[:, 2]-9.81, label='Z Acc (body)')

    #plt.plot(df['timestamp'], filter_acc[:, 0], label='X Acc (filtered)')
    #plt.plot(df['timestamp'], filter_acc[:, 1], label='Y Acc (filtered)')
    plt.plot(df['timestamp'], filter_acc[:, 2]-9.81, label='Z Acc (filtered)')

    #plt.plot(df['timestamp'], acc_world[:, 2], label='Z Acc (world)')
    #plt.plot(df['timestamp'], filter_vel[:, 2], label='Z Vel')
    #plt.plot(df['timestamp'], filter_pos[:, 2], label='Z Pos')
    
    plt.xlabel("Time (s)")
    plt.ylabel("Value")
    plt.title("Z-axis Acceleration, Velocity, and Position")
    plt.legend()
    plt.grid(True)
    plt.show()

    # anim = animate_orientation(orientations, df['timestamp'])
    # plt.show(block=True)
    # anim.save("animation.mp4")
    # plt.close()
    plot_flight_path(filter_pos)

if __name__ == '__main__':
    main()
