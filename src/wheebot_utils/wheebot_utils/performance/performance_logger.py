import pandas as pd
import matplotlib.pyplot as plt

def plot_usage(csv_file="system_usage.csv"):
    df = pd.read_csv(csv_file)
    x = range(len(df))

    plt.figure(figsize=(12, 6))
    plt.plot(x, df["cpu_percent"], label="CPU %")
    plt.plot(x, df["ram_percent"], label="RAM %")
    plt.plot(x, df["gpu_percent"], label="GPU %")
    plt.plot(x, df["vram_percent"], label="VRAM %")

    plt.xlabel("Waktu (satuan sampel)")
    plt.ylabel("Persentase (%)")
    plt.title("Monitoring Sumber Daya Sistem")
    plt.legend()
    # plt.xticks(rotation=45)
    plt.tight_layout()
    plt.grid(True)
    plt.show()

if __name__ == "__main__":
    plot_usage("system_usage.csv")
