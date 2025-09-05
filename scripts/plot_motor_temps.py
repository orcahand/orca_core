import argparse
import csv
import matplotlib.pyplot as plt
from datetime import datetime


def plot_motor_temps(logfile):
    timestamps = []
    motor_temps = {}

    with open(logfile, 'r') as f:
        reader = csv.DictReader(f)
        motor_ids = [col for col in reader.fieldnames if col.startswith('motor_')]
        for row in reader:
            timestamps.append(datetime.fromisoformat(row['timestamp']))
            for mid in motor_ids:
                if mid not in motor_temps:
                    motor_temps[mid] = []
                temp = float(row[mid]) if row[mid] != '' else None
                motor_temps[mid].append(temp)

    plt.figure(figsize=(12, 6))
    for mid in motor_ids:
        plt.plot(timestamps, motor_temps[mid], label=mid)
    plt.xlabel('Time')
    plt.ylabel('Temperature (°C)')
    plt.title('Motor Temperatures Over Time')
    plt.legend()
    plt.tight_layout()
    plt.show()


def main():
    parser = argparse.ArgumentParser(description='Plot motor temperatures from log file')
    parser.add_argument('logfile', type=str, help='Path to the motor temperature CSV log file')
    args = parser.parse_args()
    plot_motor_temps(args.logfile)


if __name__ == '__main__':
    main()
