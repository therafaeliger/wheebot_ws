#!/usr/bin/env python3
import argparse
import numpy as np
from PIL import Image
import sys

def load_pgm(path: str) -> np.ndarray:
    """Load file PGM sebagai array numpy grayscale."""
    try:
        img = Image.open(path)
        arr = np.array(img, dtype=np.uint8)
        return arr
    except Exception as e:
        print(f"Error membaca {path}: {e}", file=sys.stderr)
        sys.exit(1)

def to_binary_map(arr: np.ndarray, threshold: int = 127, invert: bool = True) -> np.ndarray:
    """
    Ubah grayscale jadi biner (occupied / free).
    - invert=True berarti 0 dianggap hitam (occupied), 255 putih (free)
    """
    if invert:
        binary = arr <= threshold
    else:
        binary = arr >= threshold
    return binary.astype(np.uint8)

def compute_iou(map1: np.ndarray, map2: np.ndarray) -> float:
    """Hitung Intersection-over-Union antara dua peta biner."""
    if map1.shape != map2.shape:
        raise ValueError(f"Ukuran berbeda: {map1.shape} vs {map2.shape}")
    
    intersection = np.logical_and(map1, map2).sum()
    union = np.logical_or(map1, map2).sum()
    if union == 0:
        return 1.0 if intersection == 0 else 0.0
    return intersection / union

def main():
    parser = argparse.ArgumentParser(description="Hitung IoU antara dua file PGM (peta 2D).")
    parser.add_argument("pgm1", help="Path ke peta pertama (PGM)")
    parser.add_argument("pgm2", help="Path ke peta kedua (PGM)")
    parser.add_argument("--threshold", type=int, default=127,
                        help="Nilai ambang binerisasi (default=127)")
    parser.add_argument("--no-invert", action="store_true",
                        help="Jangan balik warna (gunakan putih=occupied)")
    args = parser.parse_args()

    img1 = load_pgm(args.pgm1)
    img2 = load_pgm(args.pgm2)

    bin1 = to_binary_map(img1, threshold=args.threshold, invert=not args.no_invert)
    bin2 = to_binary_map(img2, threshold=args.threshold, invert=not args.no_invert)

    iou_value = compute_iou(bin1, bin2)
    print(f"IoU: {iou_value:.6f}  |  threshold={args.threshold}  |  invert={not args.no_invert}")

if __name__ == "__main__":
    main()
