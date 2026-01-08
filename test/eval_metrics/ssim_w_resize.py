#!/usr/bin/env python3
import argparse
import sys
from typing import Tuple
import numpy as np

# --- Loader: use Pillow for generic images ---
try:
    from PIL import Image
except ImportError:
    print("Error: Paket Pillow belum terpasang. Instal dengan: pip install Pillow", file=sys.stderr)
    sys.exit(1)

def load_image_as_pil(path: str) -> Image.Image:
    """Load image using Pillow and ensure Grayscale mode."""
    try:
        img = Image.open(path)
        if img.mode != 'L':
            img = img.convert('L')
        return img
    except Exception as e:
        print(f"Error loading image {path}: {e}", file=sys.stderr)
        sys.exit(1)

def to_numpy(pil_img: Image.Image) -> np.ndarray:
    """Convert PIL Image to float64 numpy array."""
    return np.array(pil_img).astype(np.float64, copy=False)

# --- SSIM Functions ---
def try_skimage_ssim(img1: np.ndarray, img2: np.ndarray, data_range: float, gaussian: bool, win_size: int) -> Tuple[bool, float]:
    try:
        from skimage.metrics import structural_similarity
        s = structural_similarity(
            img1, img2,
            data_range=data_range,
            gaussian_weights=gaussian,
            use_sample_covariance=False,
            win_size=win_size if win_size % 2 == 1 else None
        )
        return True, float(s)
    except Exception:
        return False, 0.0

def integral_image(a: np.ndarray) -> np.ndarray:
    ii = np.cumsum(np.cumsum(a, axis=0), axis=1)
    return np.pad(ii, ((1, 0), (1, 0)), mode="constant")

def box_sum(ii: np.ndarray, w: int, H: int, W: int) -> np.ndarray:
    A = ii[w:w+H,     w:w+W]
    B = ii[0:H,       w:w+W]
    C = ii[w:w+H,     0:W]
    D = ii[0:H,       0:W]
    return A - B - C + D

def ssim_uniform(img1: np.ndarray, img2: np.ndarray, data_range: float, win_size: int = 11) -> float:
    # Double check shape (safety net)
    if img1.shape != img2.shape:
        raise ValueError(f"Shape mismatch inside calc: {img1.shape} vs {img2.shape}")

    H, W = img1.shape
    pad = win_size // 2
    
    # Pad images
    x = np.pad(img1, pad_width=pad, mode='edge')
    y = np.pad(img2, pad_width=pad, mode='edge')
    n = float(win_size * win_size)

    # Integral images
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

    K1, K2 = 0.01, 0.03
    L = float(data_range)
    C1 = (K1 * L) ** 2
    C2 = (K2 * L) ** 2

    numerator   = (2.0 * mu_x * mu_y + C1) * (2.0 * sigma_xy + C2)
    denominator = (mu_x * mu_x + mu_y * mu_y + C1) * (sigma_x2 + sigma_y2 + C2)

    ssim_map = numerator / (denominator + 1e-12)
    return float(np.mean(np.clip(ssim_map, -1.0, 1.0)))

def infer_data_range(img1: np.ndarray, img2: np.ndarray, override: float = None) -> float:
    if override is not None: return float(override)
    mx = max(img1.max(), img2.max())
    if mx > 255.0: return 65535.0
    if mx > 1.0: return 255.0
    return 1.0

def main():
    parser = argparse.ArgumentParser(description="Hitung SSIM dengan Auto-Resize.")
    parser.add_argument("img1", help="Path gambar referensi (acuan ukuran)")
    parser.add_argument("img2", help="Path gambar kedua (akan di-resize jika beda)")
    parser.add_argument("--win-size", type=int, default=11)
    parser.add_argument("--data-range", type=float, default=None)
    parser.add_argument("--prefer-skimage", action="store_true")
    args = parser.parse_args()

    # 1. Load sebagai object PIL dulu (belum jadi numpy array)
    pil1 = load_image_as_pil(args.img1)
    pil2 = load_image_as_pil(args.img2)

    # 2. Cek ukuran, resize pil2 jika beda
    if pil1.size != pil2.size:
        print(f"⚠️  INFO: Beda ukuran terdeteksi!", file=sys.stderr)
        print(f"   Ref: {pil1.size} | Target: {pil2.size}", file=sys.stderr)
        print(f"   Melakukan resize pada '{args.img2}' agar sama dengan referensi...", file=sys.stderr)
        
        # Menggunakan LANCZOS untuk kualitas downscaling terbaik
        # Jika versi Pillow lama gunakan Image.ANTIALIAS
        resample_method = getattr(Image, 'LANCZOS', Image.BICUBIC)
        pil2 = pil2.resize(pil1.size, resample_method)

    # 3. Convert ke Numpy
    img1 = to_numpy(pil1)
    img2 = to_numpy(pil2)

    data_range = infer_data_range(img1, img2, args.data_range)

    used_skimage = False
    ssim_value = None

    if args.prefer_skimage:
        ok, val = try_skimage_ssim(img1, img2, data_range, gaussian=True, win_size=args.win_size)
        if ok:
            used_skimage = True
            ssim_value = val

    if ssim_value is None:
        ssim_value = ssim_uniform(img1, img2, data_range=data_range, win_size=args.win_size)

    method = "skimage" if used_skimage else "NumPy"
    print(f"SSIM: {ssim_value:.6f}  |  method={method}  |  Resized=Yes")

if __name__ == "__main__":
    main()