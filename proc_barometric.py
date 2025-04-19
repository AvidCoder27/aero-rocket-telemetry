import pandas as pd
import matplotlib.pyplot as plt
import requests
import re
from datetime import datetime
from zoneinfo import ZoneInfo

# Log file name:
LOG_DIR = "test_data_pressure/"
LOG_FILE = "bmp_4_bigger.csv"

def format_metar_time_eastern(year: int, month: int, day: int, hour: int, minute: int):
    """
    Converts Eastern Time (including DST) to METAR UTC format 'YYYYMMDDTHH:MMZ'
    """
    # Eastern time zone with automatic DST support
    eastern = ZoneInfo("America/New_York")
    local_dt = datetime(year, month, day, hour, minute, tzinfo=eastern)
    utc_dt = local_dt.astimezone(ZoneInfo("UTC"))
    return utc_dt.strftime('%Y%m%dT%H:%MZ')

def fetch_metar_pressure_temp(icao_code: str, date_time_utc: str = None):
    """
    Fetch METAR data from NOAA for a given ICAO station code.
    Optionally provide a UTC datetime string in format 'YYYYMMDDTHH:MMZ' to get past data.
    Returns: sea-level pressure (Pa), temperature (K)
    """
    base_url = "https://aviationweather.gov/api/data/metar"
    params = {
        "ids": icao_code,
        "format": "raw",
        "hours": 1
    }
    if date_time_utc:
        params["date"] = date_time_utc

    response = requests.get(base_url, params=params)
    if response.status_code != 200 or not response.text.strip():
        raise RuntimeError(f"Failed to fetch METAR data: {response.status_code} {response.text}")
    
    metar = response.text.strip().splitlines()[0]
    print(f"Fetched METAR: {metar}")

    # Extract temperature (e.g., 16/07 or M01/M02)
    temp_match = re.search(r'(\s|-)(M?\d{2})/(M?\d{2})', metar)
    if temp_match:
        temp_c = int(temp_match.group(2).replace('M', '-'))
    else:
        raise ValueError("Temperature not found in METAR")

    # Extract altimeter (e.g., A3011)
    altimeter_match = re.search(r'\sA(\d{4})', metar)
    if altimeter_match:
        altimeter_inHg = int(altimeter_match.group(1)) / 100
        pressure_pa = altimeter_inHg * 3386.389
    else:
        raise ValueError("Altimeter setting not found in METAR")

    temp_k = temp_c + 273.15
    return pressure_pa, temp_k

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

    # ICAO code for Baltimore
    icao_code = "KBWI"
    observation_time = format_metar_time_eastern(year=2025, month=4, day=18, hour=20, minute=30)

    station_pressure, station_temp = fetch_metar_pressure_temp(icao_code, observation_time)
    print(f"Station pressure: {station_pressure:.2f} Pa, Temperature: {station_temp:.2f} K")

    # Calculate altitude using barometric formula
    df['altitude'] = calculate_altitude(df['pressure'].values, pressure_at_sea=station_pressure, temp_at_sea=station_temp)

    # Print some stats
    print(f"Min altitude: {df['altitude'].min():.2f} m")
    print(f"Max altitude: {df['altitude'].max():.2f} m")
    print(f"Delta altitude: {df['altitude'].max() - df['altitude'].min():.2f} m")

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
