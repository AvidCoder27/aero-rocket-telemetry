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

# Filter out frequencies above these values
ACCEL_FILTER_HIGH = 5  # Hz
VEL_FILTER_HIGH = 3  # Hz
POS_FILTER_HIGH = 1  # Hz

# Filter out frequencies below these values
VEL_FILTER_LOW = 0.2  # Hz
POS_FILTER_LOW = 0.2  # Hz

DO_WORLD_FRAME_ROTATION = True  # Set to True to rotate the accelerometer data into the world frame

# Log file name:
LOG_DIR = "test_data/"
LOG_FILE = "log_6_fake_launches.csv"

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

# removes high frequency noise
def lowpass_filter(data, fs, cutoff, order=4):
    b, a = butter(order, cutoff, btype='lowpass', fs=fs)
    return filtfilt(b, a, data)

# removes low frequency noise
def highpass_filter(data, fs, cutoff, order=4):
    b, a = butter(order, cutoff, btype='highpass', fs=fs)
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
    filter_acc_x = lowpass_filter(acc_body[:, 0], fs, cutoff=ACCEL_FILTER_HIGH)
    filter_acc_y = lowpass_filter(acc_body[:, 1], fs, cutoff=ACCEL_FILTER_HIGH)
    filter_acc_z = lowpass_filter(acc_body[:, 2], fs, cutoff=ACCEL_FILTER_HIGH)
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
    velocity = cumulative_trapezoid(acc_world, timestamp, initial=0, axis=0)

    # remove high frequcy noise from velocity
    filter_vel_x = lowpass_filter(velocity[:, 0], fs, cutoff=VEL_FILTER_HIGH)
    filter_vel_y = lowpass_filter(velocity[:, 1], fs, cutoff=VEL_FILTER_HIGH)
    filter_vel_z = lowpass_filter(velocity[:, 2], fs, cutoff=VEL_FILTER_HIGH)

    # remove low frequcy noise from velocity that comes from integration drift
    filter_vel_x = highpass_filter(filter_vel_x, fs, cutoff=VEL_FILTER_LOW)
    filter_vel_y = highpass_filter(filter_vel_y, fs, cutoff=VEL_FILTER_LOW)
    filter_vel_z = highpass_filter(filter_vel_z, fs, cutoff=VEL_FILTER_LOW)

    # recombine the dims after filtering
    filter_vel = np.column_stack((filter_vel_x, filter_vel_y, filter_vel_z))

    # Use Zero Velocity Update (ZUPT) to remove drift
    # This is done by getting the velocity bias of the stationary rocket
    # vel_bias = filter_vel[0:20].mean(axis=0)
    # filter_vel -= vel_bias

    # Integrate the filtered velocity to get position
    position = cumulative_trapezoid(filter_vel, timestamp, initial=0, axis=0)

    # Filter the position with another butterworth filter but at a lower cutoff frequency
    filter_pos_x = lowpass_filter(position[:, 0], fs, cutoff=POS_FILTER_HIGH)
    filter_pos_y = lowpass_filter(position[:, 1], fs, cutoff=POS_FILTER_HIGH)
    filter_pos_z = lowpass_filter(position[:, 2], fs, cutoff=POS_FILTER_HIGH)

    # remove low frequcy noise from position that comes from integration drift
    filter_pos_x = highpass_filter(filter_pos_x, fs, cutoff=POS_FILTER_LOW)
    filter_pos_y = highpass_filter(filter_pos_y, fs, cutoff=POS_FILTER_LOW)
    filter_pos_z = highpass_filter(filter_pos_z, fs, cutoff=POS_FILTER_LOW)
    
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
    acc_raw, gyro, orientations, acc_filter, acc_world, velocity, filter_vel, position, filter_pos = integrate_motion(df)

    # Debug plotting

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

    # Truncate the data to a specific time window for debugging
    START_TIME = 15.0
    END_TIME = 22.0
    mask = (time > START_TIME) & (time < END_TIME)

    x_acc_raw = x_acc_raw[mask]
    y_acc_raw = y_acc_raw[mask]
    z_acc_raw = z_acc_raw[mask]
    x_acc_filtered = x_acc_filtered[mask]
    y_acc_filtered = y_acc_filtered[mask]
    z_acc_filtered = z_acc_filtered[mask]
    x_acc_world = x_acc_world[mask]
    y_acc_world = y_acc_world[mask]
    z_acc_world = z_acc_world[mask]
    x_vel = x_vel[mask]
    y_vel = y_vel[mask]
    z_vel = z_vel[mask]
    x_vel_filt = x_vel_filt[mask]
    y_vel_filt = y_vel_filt[mask]
    z_vel_filt = z_vel_filt[mask]
    x_pos = x_pos[mask]
    y_pos = y_pos[mask]
    z_pos = z_pos[mask]
    x_pos_filt = x_pos_filt[mask]
    y_pos_filt = y_pos_filt[mask]
    z_pos_filt = z_pos_filt[mask]
    time = time[mask]

    plt.figure()
    # plt.plot(time, x_acc_raw, label='X Accel Raw')
    # plt.plot(time, y_acc_raw, label='Y Accel Raw')
    # plt.plot(time, z_acc_raw, label='Z Accel Raw')

    # plt.plot(time, x_acc_filtered, label='X Accel Filtered')
    # plt.plot(time, y_acc_filtered, label='Y Accel Filtered')
    # plt.plot(time, z_acc_filtered, label='Z Accel Filtered')

    # plt.plot(time, x_acc_world, label='X Accel World')
    # plt.plot(time, y_acc_world, label='Y Accel World')
    # plt.plot(time, z_acc_world, label='Z Accel World')

    # plt.plot(time, x_vel, label='X Vel')
    # plt.plot(time, y_vel, label='Y Vel')
    # plt.plot(time, z_vel, label='Z Vel')

    # plt.plot(time, x_vel_filt, label='X Vel Filtered')
    # plt.plot(time, y_vel_filt, label='Y Vel Filtered')
    # plt.plot(time, z_vel_filt, label='Z Vel Filtered')

    # plt.plot(time, x_pos, label='X Pos')
    # plt.plot(time, y_pos, label='Y Pos')
    # plt.plot(time, z_pos, label='Z Pos')

    plt.plot(time, x_pos_filt, label='X Pos Filtered')
    plt.plot(time, y_pos_filt, label='Y Pos Filtered')
    plt.plot(time, z_pos_filt, label='Z Pos Filtered')
    
    plt.xlabel("Time (s)")
    plt.ylabel("Value")
    plt.title("Debugging IMU Data")
    plt.legend()
    plt.grid(True)
    plt.show()

    # anim = animate_orientation(orientations, df['timestamp'])
    # anim.save("animation.mp4")
    plot_flight_path(filter_pos)

if __name__ == '__main__':
    main()
