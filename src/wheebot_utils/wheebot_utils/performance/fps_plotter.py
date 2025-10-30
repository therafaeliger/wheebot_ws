#!/usr/bin/env python3
import matplotlib.pyplot as plt
import csv
import sys
from datetime import datetime

def plot_fps(csv_file):
    timestamps = []
    fps_values = []

    with open(csv_file, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            timestamps.append(datetime.strptime(row['timestamp'], "%Y-%m-%d %H:%M:%S.%f"))
            fps_values.append(float(row['fps']))

    plt.figure(figsize=(10,5))
    plt.plot(timestamps, fps_values, label='FPS', linewidth=2)
    plt.xlabel("Waktu")
    plt.ylabel("Frames per Second (FPS)")
    plt.title("Monitoring FPS YOLO Node")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python fps_plotter.py <fps_log.csv>")
        sys.exit(1)
    plot_fps(sys.argv[1])
