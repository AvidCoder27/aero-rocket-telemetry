import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import requests
import re
from datetime import datetime
from zoneinfo import ZoneInfo

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
    Fetch METAR data from NOAA for a given ICAO station code and date/time.
    Returns: sea-level pressure (Pa), temperature (K), dew point (Celsius)
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

    # Extract temperature and dew point
    # Temperature and Dew Point: (e.g., 16/07, 15/M03)
    temp_match = re.search(r'(\s|-)(M?\d{2})/(M?\d{2})', metar)
    if temp_match:
        temp_c = int(temp_match.group(2).replace('M', '-'))
        dew_point_c = int(temp_match.group(3).replace('M', '-'))
    else:
        raise ValueError("Temperature and dew point not found in METAR")

    # Extract altimeter setting (e.g., A3011)
    altimeter_match = re.search(r'\sA(\d{4})', metar)
    if altimeter_match:
        altimeter_inHg = int(altimeter_match.group(1)) / 100
        pressure_pa = altimeter_inHg * 3386.389  # Convert to Pa
    else:
        raise ValueError("Altimeter setting not found in METAR")

    temp_k = temp_c + 273.15
    return pressure_pa, temp_k, dew_point_c

def calculate_virtual_temperature(temp_k: float, dew_point_c: float, pressure_pa: float):
    """
    Calculates the virtual temperature accounting for humidity.
    - temp_k: actual temperature (Kelvin)
    - dew_point_c: dew point (Celsius)
    - pressure_pa: pressure in Pascals

    Returns:
        Virtual temperature (Kelvin)
    """
    # Convert dew point to vapor pressure (e) in hPa
    e = 6.112 * np.exp((17.67 * dew_point_c) / (dew_point_c + 243.5))  # hPa

    # Convert pressure to hPa
    pressure_hpa = pressure_pa / 100.0

    # Virtual temp correction
    Tv = temp_k * (1 + 0.61 * (e / pressure_hpa))

    return Tv

def calculate_altitude(pressures, pressure_at_sea: float, temp_at_sea: float):
    """
    Calculates altitude from pressure readings using the barometric formula.
    - pressures: array of pressure values (Pa)
    - pressure_at_sea: reference sea-level pressure (Pa)
    - temp_at_sea: temperature at sea level (K)
    
    Returns:
        altitudes: array of altitude estimates (m)
    """
    # Constants
    LAPSE_RATE = 0.0065  # K/m (positive for math)
    GAS_CONSTANT = 8.31432  # J/(mol*K)
    MOLAR_MASS = 0.0289644  # kg/mol
    GRAVITY = 9.80665  # m/s²

    exponent = (GAS_CONSTANT * LAPSE_RATE) / (GRAVITY * MOLAR_MASS)

    # Ensure numpy array
    pressures = np.asarray(pressures)

    # Prevent divide-by-zero or negative pressures
    pressures = np.clip(pressures, 1, None)

    # Barometric formula with lapse rate
    altitudes = (temp_at_sea / LAPSE_RATE) * (1 - (pressures / pressure_at_sea) ** exponent)

    return altitudes

def main():
    # Load the data
    filename = LOG_DIR + LOG_FILE
    df = load_data(filename)

    # ICAO code for Baltimore
    icao_code = "KBWI"
    observation_time = format_metar_time_eastern(year=2025, month=4, day=18, hour=20, minute=30)

    station_pressure, station_temp, station_dew_point = fetch_metar_pressure_temp(icao_code, observation_time)
    virtual_temp = calculate_virtual_temperature(station_temp, station_dew_point, station_pressure)
    print(f"Station pressure: {station_pressure:.2f} Pa, Temperature: {station_temp:.2f} K, Dew Point: {station_dew_point:.2f} °C, Virtual Temp: {virtual_temp:.2f} K")

    # Calculate altitude using barometric formula
    altitude = calculate_altitude(df['pressure'].values, pressure_at_sea=station_pressure, temp_at_sea=virtual_temp)

    # Print some stats
    print(f"Min altitude: {altitude.min():.2f} m")
    print(f"Max altitude: {altitude.max():.2f} m")
    print(f"Delta altitude: {altitude.max() - altitude.min():.2f} m")

    # Plot the data
    plt.figure(figsize=(10, 6))
    plt.plot(df['timestamp'], altitude)
    plt.xlabel('Time (s)')
    plt.ylabel('Altitude (m)')
    plt.title('Altitude vs Time')
    plt.grid(True)
    plt.show()

if __name__ == '__main__':
    main()
