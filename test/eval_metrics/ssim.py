#!/usr/bin/env python3
import argparse
import sys
from typing import Tuple
import numpy as np

# --- Loader: use Pillow for generic images (PNG, JPG, PGM, etc.) ---
try:
    from PIL import Image
except ImportError:
    print("Error: Paket Pillow belum terpasang. Instal dengan: pip install Pillow", file=sys.stderr)
    sys.exit(1)

def load_image(path: str) -> np.ndarray:
    """
    Load image (PNG, JPG, PGM) as grayscale ndarray (float64).
    Auto converts RGB to Grayscale ('L') to ensure 2D structure.
    """
    try:
        img = Image.open(path)
        
        # Konversi ke grayscale (L) jika gambar berwarna (RGB/RGBA)
        # SSIM standar bekerja pada channel luma (intensitas)
        if img.mode != 'L':
            img = img.convert('L')
            
        arr = np.array(img)
        # convert to float64 for numeric stability
        return arr.astype(np.float64, copy=False)
    except Exception as e:
        print(f"Error loading image {path}: {e}", file=sys.stderr)
        sys.exit(1)

# --- Optional: skimage SSIM if available ---
def try_skimage_ssim(img1: np.ndarray, img2: np.ndarray, data_range: float, gaussian: bool, win_size: int) -> Tuple[bool, float]:
    try:
        from skimage.metrics import structural_similarity
        s = structural_similarity(
            img1, img2,
            data_range=data_range,
            gaussian_weights=gaussian,
            use_sample_covariance=False,
            win_size=win_size if win_size % 2 == 1 else None  # skimage needs odd win_size
        )
        return True, float(s)
    except Exception:
        return False, 0.0

# --- Pure NumPy SSIM using integral images (uniform window) ---
def integral_image(a: np.ndarray) -> np.ndarray:
    ii = np.cumsum(np.cumsum(a, axis=0), axis=1)
    # pad top/left with a zero border for easier box-sum indexing
    return np.pad(ii, ((1, 0), (1, 0)), mode="constant")

def box_sum(ii: np.ndarray, w: int, H: int, W: int) -> np.ndarray:
    # Sum over all WxW windows with top-left positions covering the original HxW
    A = ii[w:w+H,     w:w+W]
    B = ii[0:H,       w:w+W]
    C = ii[w:w+H,     0:W]
    D = ii[0:H,       0:W]
    return A - B - C + D

def ssim_uniform(img1: np.ndarray, img2: np.ndarray, data_range: float, win_size: int = 11) -> float:
    if img1.shape != img2.shape:
        raise ValueError(f"Beda ukuran gambar: {img1.shape} vs {img2.shape}")

    if win_size % 2 == 0 or win_size < 3:
        raise ValueError("win_size harus ganjil dan >= 3 (mis. 11)")

    H, W = img1.shape
    pad = win_size // 2

    # Edge replicate pad agar jendela valid di semua piksel
    x = np.pad(img1, pad_width=pad, mode='edge')
    y = np.pad(img2, pad_width=pad, mode='edge')

    n = float(win_size * win_size)

    # Integral images for x, y, x^2, y^2, xy
    ii_x   = integral_image(x)
    ii_y   = integral_image(y)
    ii_x2  = integral_image(x * x)
    ii_y2  = integral_image(y * y)
    ii_xy  = integral_image(x * y)

    sum_x  = box_sum(ii_x,  win_size, H, W)
    sum_y  = box_sum(ii_y,  win_size, H, W)
    sum_x2 = box_sum(ii_x2, win_size, H, W)
    sum_y2 = box_sum(ii_y2, win_size, H, W)
    sum_xy = box_sum(ii_xy, win_size, H, W)

    mu_x = sum_x / n
    mu_y = sum_y / n

    sigma_x2 = (sum_x2 / n) - (mu_x * mu_x)
    sigma_y2 = (sum_y2 / n) - (mu_y * mu_y)
    sigma_xy = (sum_xy / n) - (mu_x * mu_y)

    # SSIM constants
    K1, K2 = 0.01, 0.03
    L = float(data_range)
    C1 = (K1 * L) ** 2
    C2 = (K2 * L) ** 2

    numerator   = (2.0 * mu_x * mu_y + C1) * (2.0 * sigma_xy + C2)
    denominator = (mu_x * mu_x + mu_y * mu_y + C1) * (sigma_x2 + sigma_y2 + C2)

    ssim_map = numerator / (denominator + 1e-12)
    ssim_map = np.clip(ssim_map, -1.0, 1.0)

    return float(np.mean(ssim_map))

def infer_data_range(img1: np.ndarray, img2: np.ndarray, override: float = None) -> float:
    if override is not None:
        return float(override)
    # Heuristic: use dtype-based range if integer; else use max-min across both
    # Note: img1 loaded via PIL convert('L') usually results in generic float after astype
    # but the original values were likely 0-255.
    
    current_max = max(img1.max(), img2.max())
    if current_max > 1.0:
        # Kemungkinan besar range 0-255 (8-bit) atau 0-65535 (16-bit)
        if current_max > 255:
            return 65535.0
        return 255.0
    
    # Jika nilainya ternormalisasi 0.0-1.0
    return 1.0

def main():
    parser = argparse.ArgumentParser(
        description="Hitung SSIM antara dua gambar (PNG/JPG/PGM) dan cetak ke terminal."
    )
    parser.add_argument("img1", help="Path ke gambar pertama (Reference)")
    parser.add_argument("img2", help="Path ke gambar kedua (Distorted)")
    parser.add_argument("--win-size", type=int, default=11, help="Ukuran jendela (ganjil), default=11")
    parser.add_argument("--data-range", type=float, default=None,
                        help="Range intensitas (L). Jika kosong, akan ditebak (biasanya 255).")
    parser.add_argument("--prefer-skimage", action="store_true",
                        help="Jika tersedia, pakai skimage (Gaussian window).")
    args = parser.parse_args()

    img1 = load_image(args.img1)
    img2 = load_image(args.img2)

    # Validasi dimensi (harusnya sudah 2D karena convert L)
    if img1.ndim != 2 or img2.ndim != 2:
        print("Error: Gambar harus 2D (Grayscale).", file=sys.stderr)
        sys.exit(2)

    if img1.shape != img2.shape:
        print(f"Error: Ukuran tidak sama: {img1.shape} vs {img2.shape}. Pastikan resolusi sama.", file=sys.stderr)
        sys.exit(2)

    data_range = infer_data_range(img1, img2, args.data_range)

    used_skimage = False
    ssim_value = None

    if args.prefer_skimage:
        ok, val = try_skimage_ssim(img1, img2, data_range, gaussian=True, win_size=args.win_size)
        if ok:
            used_skimage = True
            ssim_value = val

    if ssim_value is None:
        # fallback: pure numpy, uniform window
        ssim_value = ssim_uniform(img1, img2, data_range=data_range, win_size=args.win_size)

    # Print result
    method = "skimage (Gaussian)" if used_skimage else "NumPy (uniform window)"
    print(f"SSIM: {ssim_value:.6f}  |  method={method}  |  win_size={args.win_size}  |  data_range={data_range:g}")

if __name__ == "__main__":
    main()