import pandas as pd
import matplotlib.pyplot as plt

# Log file name:
LOG_DIR = "test_data_pressure/"
LOG_FILE = "bmp_4_bigger.csv"

def load_data(filename):
    df = pd.read_csv(filename, header=None)
    df.columns = ['timestamp', 'pressure', 'temperature']
    df['timestamp'] = df['timestamp'] * 1e-3  # Convert ms to seconds
    df['pressure'] = df['pressure'] * 100 # Convert hPa to Pa
    df['temperature'] = df['temperature'] + 273.15 # Convert to Kelvin

    df = df.sort_values(by='timestamp')
    df['timestamp'] = df['timestamp'] - df['timestamp'].iloc[0]  # start at 0
    df['dt'] = df['timestamp'].diff().fillna(0)
    return df

def calculate_altitude(pressures, pressure_at_sea: float, temp_at_sea: float):
    """
    Calculate the altitude based on the pressure and temperature at sea level.
    Pressure is in Pa, temperature is in Kelvin.
    """
    # Scroll down to see the formula: (https://www.mide.com/air-pressure-at-altitude-calculator)

    LAPSE_RATE = -0.0065  # Kelvin/meter
    UNIVERSAL_GAS_CONSTANT = 8.31432  # N*m/(mol*K)
    MOLAR_MASS_AIR = 0.0289644  # kg/mol
    GRAVITY = 9.80665  # m/s^2
    hb = 0  # "height at the bottom of atmospheric layer [m]"
    altitudes = hb + temp_at_sea / LAPSE_RATE * ((pressures / pressure_at_sea) ** (-UNIVERSAL_GAS_CONSTANT * LAPSE_RATE / GRAVITY / MOLAR_MASS_AIR) - 1)
    return altitudes

def main():
    # Load the data
    filename = LOG_DIR + LOG_FILE
    df = load_data(filename)
    df['altitude'] = calculate_altitude(df['pressure'].values, 101710, 21+273.15)
    # Plot the data
    plt.figure(figsize=(10, 6))
    plt.plot(df['timestamp'], df['altitude'])
    plt.xlabel('Time (s)')
    plt.ylabel('Altitude (m)')
    plt.title('Altitude vs Time')
    plt.grid(True)
    plt.show()

if __name__ == '__main__':
    main()
