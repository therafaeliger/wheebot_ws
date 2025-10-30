import psutil
import time
import csv
import datetime
import subprocess
import re

def get_gpu_usage():
    """Ambil data GPU usage (NVIDIA GPU via nvidia-smi)."""
    try:
        result = subprocess.check_output(
            ["nvidia-smi", "--query-gpu=utilization.gpu,memory.used,memory.total",
             "--format=csv,noheader,nounits"],
            encoding="utf-8"
        )
        gpu_util, mem_used, mem_total = map(float, result.strip().split(','))
        return gpu_util, mem_used / mem_total * 100  # GPU %, VRAM %
    except Exception:
        return 0.0, 0.0  # Jika tidak ada GPU NVIDIA

def monitor_system(output_file="/home/rafael/wheebot_ws/src/wheebot_utils/results/system_usage.csv", interval=1):
    """Monitoring sistem tiap detik dan simpan ke CSV."""
    fieldnames = ["timestamp", "cpu_percent", "ram_percent", "gpu_percent", "vram_percent"]

    with open(output_file, mode="w", newline="") as csvfile:
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        writer.writeheader()

        print("Monitoring system... Tekan Ctrl+C untuk berhenti.\n")
        try:
            while True:
                cpu = psutil.cpu_percent(interval=None)
                ram = psutil.virtual_memory().percent
                gpu, vram = get_gpu_usage()

                timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")

                # Tampilkan di terminal
                print(f"[{timestamp}] CPU: {cpu:5.1f}% | RAM: {ram:5.1f}% | GPU: {gpu:5.1f}% | VRAM: {vram:5.1f}%")

                # Simpan ke file
                writer.writerow({
                    "timestamp": timestamp,
                    "cpu_percent": cpu,
                    "ram_percent": ram,
                    "gpu_percent": gpu,
                    "vram_percent": vram
                })
                csvfile.flush()
                time.sleep(interval)

        except KeyboardInterrupt:
            print("\nMonitoring dihentikan.")
            print(f"Data tersimpan di: {output_file}")

if __name__ == "__main__":
    monitor_system(output_file="/home/rafael/wheebot_ws/src/wheebot_utils/results/system_usage.csv", interval=1)
