import argparse
import csv
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from datetime import datetime, timedelta
import time
import os
import glob
import numpy as np


def find_latest_csv(logs_dir):
    """Find the most recently created CSV file in the logs directory"""
    csv_pattern = os.path.join(logs_dir, "*.csv")
    csv_files = glob.glob(csv_pattern)
    
    if not csv_files:
        return None
    
    # Get the most recently created file
    latest_file = max(csv_files, key=os.path.getctime)
    return latest_file


def calculate_moving_average(data, window_size):
    """Calculate moving average with the specified window size"""
    if len(data) < window_size:
        return data
    
    weights = np.ones(window_size) / window_size
    return np.convolve(data, weights, mode='valid')


def plot_motor_temps_live(logfile, window_size=10):
    # Set up the plot style for a clean, professional look
    plt.style.use('default')
    
    fig, ax = plt.subplots(figsize=(14, 8))
    
    # Initialize data storage
    timestamps = []
    motor_temps = {}
    motor_ids = []
    
    # Track the last file size to detect updates
    last_size = 0
    
    # Define a nice color palette and line styles
    colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', '#8c564b', '#e377c2', '#7f7f7f', '#bcbd22', '#17becf']
    line_styles = ['-', '--', ':', '-.', '-', '--', ':', '-.', '-', '--']
    
    def animate(frame):
        nonlocal last_size, motor_ids
        
        # Check if file has been updated
        if not os.path.exists(logfile):
            return
        
        current_size = os.path.getsize(logfile)
        if current_size == last_size:
            return
        
        last_size = current_size
        
        # Clear previous plot
        ax.clear()
        
        # Read the updated CSV file
        try:
            with open(logfile, 'r') as f:
                reader = csv.DictReader(f)
                if not motor_ids:  # First time reading
                    motor_ids = [col for col in reader.fieldnames if col.startswith('motor_')]
                
                # Reset data for new plot
                timestamps.clear()
                for mid in motor_ids:
                    motor_temps[mid] = []
                
                # Read all rows
                for row in reader:
                    try:
                        timestamp = datetime.fromisoformat(row['timestamp'])
                        timestamps.append(timestamp)
                        for mid in motor_ids:
                            temp = float(row[mid]) if row[mid] != '' else None
                            motor_temps[mid].append(temp)
                    except (ValueError, KeyError):
                        continue
                
                # Plot the data with clean styling
                for i, mid in enumerate(motor_ids):
                    if motor_temps[mid]:  # Only plot if we have data
                        color = colors[i % len(colors)]
                        line_style = line_styles[i % len(line_styles)]
                        
                        # Filter out None values for plotting
                        valid_temps = [t for t in motor_temps[mid] if t is not None]
                        valid_timestamps = [t for t, temp in zip(timestamps, motor_temps[mid]) if temp is not None]
                        
                        if len(valid_temps) > 0:
                            # Plot only moving average with distinct styling
                            if len(valid_temps) >= window_size:
                                ma_temps = calculate_moving_average(valid_temps, window_size)
                                ma_timestamps = valid_timestamps[window_size-1:]
                                ax.plot(ma_timestamps, ma_temps, 
                                       color=color, 
                                       linestyle=line_style,
                                       linewidth=2.0, 
                                       alpha=0.9,
                                       label=f"{mid.replace('motor_', 'Motor ')}")
                
                # Customize the plot appearance
                ax.set_xlabel('Time', fontsize=12, fontweight='bold')
                ax.set_ylabel('Temperature (°C)', fontsize=12, fontweight='bold')
                ax.set_title(f'Motor Temperatures Over Time (Live) - Moving Average Window: {window_size}\n{os.path.basename(logfile)}', 
                           fontsize=14, fontweight='bold', pad=20)
                
                # Style the legend
                ax.legend(loc='upper left', frameon=True, fancybox=True, 
                         shadow=True, fontsize=9, framealpha=0.9)
                
                # Style the grid
                ax.grid(True, alpha=0.3, linestyle='-', linewidth=0.5)
                ax.set_axisbelow(True)
                
                # Style the axes
                ax.spines['top'].set_visible(False)
                ax.spines['right'].set_visible(False)
                ax.spines['left'].set_linewidth(1.5)
                ax.spines['bottom'].set_linewidth(1.5)
                
                # Rotate x-axis labels for better readability
                plt.setp(ax.get_xticklabels(), rotation=45, ha='right', fontsize=10)
                plt.setp(ax.get_yticklabels(), fontsize=10)
                
                # Auto-scale axes to show all data
                if timestamps:
                    # Set x-axis to show all data from start to finish
                    ax.set_xlim(timestamps[0], timestamps[-1])
                    
                    # Set y-axis to always show 20-90°C range
                    ax.set_ylim(20, 90)
                
                # Add horizontal reference lines
                ax.axhline(y=80, color='black', linestyle='--', linewidth=1.5, alpha=0.7, label='XC430-W210 Threshold')
                ax.axhline(y=70, color='black', linestyle='--', linewidth=1.5, alpha=0.7, label='XC330-T288 Threshold')
                
                # Set background color
                ax.set_facecolor('#f8f9fa')
                fig.patch.set_facecolor('white')
                
                plt.tight_layout()
                
        except Exception as e:
            print(f"Error reading file: {e}")
    
    # Set up animation
    ani = animation.FuncAnimation(fig, animate, interval=1000, blit=False)  # Update every 1 second
    
    plt.show()


def main():
    parser = argparse.ArgumentParser(description='Plot motor temperatures live from the latest CSV log file')
    parser.add_argument('--logfile', type=str, help='Optional: Path to a specific motor temperature CSV log file')
    parser.add_argument('--logs_dir', type=str, default='logs', help='Directory containing log files (default: logs)')
    parser.add_argument('--window', type=int, default=10, help='Moving average window size (default: 10)')
    args = parser.parse_args()
    
    # Determine which log file to use
    if args.logfile:
        # Use the specified log file
        logfile = args.logfile
        if not os.path.exists(logfile):
            print(f"Error: Specified log file '{logfile}' not found.")
            return
    else:
        # Find the latest CSV file in the logs directory
        script_dir = os.path.dirname(os.path.abspath(__file__))
        project_root = os.path.abspath(os.path.join(script_dir, '..'))
        logs_dir = os.path.join(project_root, args.logs_dir)
        
        if not os.path.exists(logs_dir):
            print(f"Error: Logs directory '{logs_dir}' not found.")
            return
        
        logfile = find_latest_csv(logs_dir)
        if not logfile:
            print(f"Error: No CSV files found in '{logs_dir}'.")
            return
        
        print(f"Found latest log file: {os.path.basename(logfile)}")
    
    print(f"Starting live plot for: {logfile}")
    print(f"Moving average window size: {args.window}")
    print("The plot will update automatically as new data is written to the CSV file.")
    print("Press Ctrl+C to stop.")
    
    try:
        plot_motor_temps_live(logfile, args.window)
    except KeyboardInterrupt:
        print("\nLive plotting stopped.")


if __name__ == '__main__':
    main()
