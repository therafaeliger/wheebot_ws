#!/usr/bin/env python3
import cv2
import numpy as np
import argparse

def load_gray(path):
    img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise FileNotFoundError(f"Gagal membaca file {path}")
    return img.astype(np.float32) / 255.0  # normalisasi 0–1

def compute_rdr(static_img, dynamic_img, mask=None, threshold=None):
    if static_img.shape != dynamic_img.shape:
        raise ValueError("Ukuran gambar tidak sama!")

    residual = np.abs(dynamic_img - static_img)

    if mask is None:
        if threshold is None:
            # gunakan threshold otomatis Otsu pada residual
            _, mask = cv2.threshold((residual * 255).astype(np.uint8), 0, 1, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        else:
            mask = (residual > threshold).astype(np.uint8)
    else:
        mask = (mask > 0).astype(np.uint8)

    total_residual = np.sum(residual)
    dynamic_residual = np.sum(residual * mask)

    rdr = dynamic_residual / total_residual if total_residual > 0 else 0.0
    return rdr, residual, mask

def main():
    parser = argparse.ArgumentParser(description="Hitung Residual Dynamic Ratio (RDR) antara frame kosong dan dinamis.")
    parser.add_argument("static_frame", help="Path ke frame referensi (tanpa orang)")
    parser.add_argument("dynamic_frame", help="Path ke frame dengan orang")
    parser.add_argument("--mask", help="Path ke mask dinamis (opsional)")
    parser.add_argument("--threshold", type=float, default=None, help="Threshold residual (0-1) jika tanpa mask")
    args = parser.parse_args()

    static_img = load_gray(args.static_frame)
    dynamic_img = load_gray(args.dynamic_frame)
    mask_img = load_gray(args.mask) if args.mask else None

    rdr, residual, mask = compute_rdr(static_img, dynamic_img, mask_img, args.threshold)

    print(f"Residual Dynamic Ratio (RDR): {rdr:.6f}")
    print(f"Semakin kecil RDR → semakin baik hasil DOR (lebih sedikit residual di area dinamis)")

    # Visualisasi hasil (opsional)
    vis = np.hstack([
        static_img,
        dynamic_img,
        cv2.normalize(residual, None, 0, 1, cv2.NORM_MINMAX),
        mask.astype(np.float32)
    ])
    cv2.imshow("Static | Dynamic | Residual | Mask", vis)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
