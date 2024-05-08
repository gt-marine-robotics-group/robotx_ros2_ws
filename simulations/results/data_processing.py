import json
import pandas as pd
import re
from datetime import datetime, timedelta
import matplotlib.pyplot as plt

def load_flight_data(file_path):
    """
    Load flight data from a JSON file.
    """
    try:
        with open(file_path, 'r') as file:
            flight_data = json.load(file)
            return flight_data
    except FileNotFoundError:
        print(f"No such file: '{file_path}'. Please check the file path.")
        return None
    except json.JSONDecodeError:
        print("Error decoding JSON. Please check the file content.")
        return None
    except Exception as e:
        print(f"An error occurred: {e}")
        return None

def parse_time(time_str):
    """
    Parse time from a custom string format to a datetime object.
    """
    try:
        sec_match = re.search(r"sec=(\d+)", time_str)
        nanosec_match = re.search(r"nanosec=(\d+)", time_str)
        
        if sec_match and nanosec_match:
            sec = int(sec_match.group(1))
            nanosec = int(nanosec_match.group(1))
            return datetime.fromtimestamp(sec) + timedelta(seconds=nanosec * 1e-9)
        else:
            print("No valid 'sec' or 'nanosec' found in time string.")
            return None
    except Exception as e:
        print(f"Error parsing time: {e}")
        return None

def load_flight_data_to_dataframe(file_path):
    """
    Load flight data from a JSON file and convert it to a pandas DataFrame with normalized time.
    """
    data = load_flight_data(file_path)
    
    if data is None:
        return None
    
    df = pd.DataFrame(data)
    df['time'] = df['time'].apply(parse_time)
    
    if df['time'].isnull().any():
        print("Some timestamps could not be parsed. Please check the data.")
        return df
    
    t0 = df['time'].iloc[0]
    df['time'] = df['time'].apply(lambda x: (x - t0).total_seconds())
    
    return df

def plot_controls_and_metrics(data_df):
    # Creating a figure with 3 subplots for controls
    fig, axs = plt.subplots(3, 1, figsize=(10, 12))
    fig.suptitle('Control Variables Over Time')

    # Plot control_x against time
    axs[0].plot(data_df['time'].values, data_df['control_x'].values, label='Control X', color='b')
    axs[0].set_title('Control X Over Time')
    axs[0].set_xlabel('Time (seconds)')
    axs[0].set_ylabel('Control X')
    axs[0].grid(True)

    # Plot control_y against time
    axs[1].plot(data_df['time'].values, data_df['control_y'].values, label='Control Y', color='r')
    axs[1].set_title('Control Y Over Time')
    axs[1].set_xlabel('Time (seconds)')
    axs[1].set_ylabel('Control Y')
    axs[1].grid(True)

    # Plot control_z against time
    axs[2].plot(data_df['time'].values, data_df['control_z'].values, label='Control Z', color='g')
    axs[2].set_title('Control Z Over Time')
    axs[2].set_xlabel('Time (seconds)')
    axs[2].set_ylabel('Control Z')
    axs[2].grid(True)

    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    plt.show()

    # Creating a figure with 2 subplots for altitude and distance
    fig, ax1 = plt.subplots(1, 2, figsize=(12, 6))
    fig.suptitle('Altitude and Distance Over Time')

    # Altitude against time
    ax1[0].plot(data_df['time'].values, data_df['altitude'].values, label='Altitude', color='magenta')
    ax1[0].set_title('Altitude Over Time')
    ax1[0].set_xlabel('Time (seconds)')
    ax1[0].set_ylabel('Altitude (meters)')
    ax1[0].grid(True)

    # Distance against time
    ax1[1].plot(data_df['time'].values, data_df['distance'].values, label='Distance', color='cyan')
    ax1[1].set_title('Distance Over Time')
    ax1[1].set_xlabel('Time (seconds)')
    ax1[1].set_ylabel('Distance (meters)')
    ax1[1].grid(True)

    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    plt.show()

# Load your data into a DataFrame
file_path = 'flight_data.json'  # Adjust this path to where your JSON data is located
data_df = load_flight_data_to_dataframe(file_path)

# Plot the data
if data_df is not None:
    plot_controls_and_metrics(data_df)
else:
    print("Data could not be loaded or parsed.")
