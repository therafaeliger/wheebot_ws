#!/usr/bin/env python3
import numpy as np
from scipy.spatial.transform import Rotation
import argparse

def load_tum_trajectory(path):
    data = []
    with open(path, "r") as f:
        for line in f:
            if line.strip().startswith("#") or len(line.strip()) == 0:
                continue
            vals = line.strip().split()
            if len(vals) < 8:
                continue
            t = float(vals[0])
            tx, ty, tz = map(float, vals[1:4])
            qx, qy, qz, qw = map(float, vals[4:8])
            data.append((t, np.array([tx, ty, tz]), np.array([qx, qy, qz, qw])))
    return np.array(data, dtype=object)

def align_trajectories(ref_xyz, est_xyz):
    """Align est_xyz ke ref_xyz via least-squares rigid transform (SVD)."""
    mu_ref = np.mean(ref_xyz, axis=0)
    mu_est = np.mean(est_xyz, axis=0)
    ref_centered = ref_xyz - mu_ref
    est_centered = est_xyz - mu_est
    H = est_centered.T @ ref_centered
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T
    t = mu_ref - R @ mu_est
    est_aligned = (R @ est_xyz.T).T + t
    return est_aligned, R, t

def compute_ate(ref_xyz, est_xyz):
    est_aligned, R, t = align_trajectories(ref_xyz, est_xyz)
    errors = np.linalg.norm(ref_xyz - est_aligned, axis=1)
    rmse = np.sqrt(np.mean(errors ** 2))
    mae = np.mean(np.abs(errors))

    # optional
    # import matplotlib.pyplot as plt
    # plt.plot(ref_xyz[:,0], ref_xyz[:,1], 'g-', label="Reference")
    # plt.plot(est_xyz[:,0], est_xyz[:,1], 'r--', label="Estimated (aligned)")
    # plt.axis('equal')
    # plt.legend()
    # plt.title("Trajectory Comparison")
    # plt.show()

    return rmse, mae, errors

def associate_trajectories(traj_ref, traj_est, max_diff=0.02):
    """Cocokkan timestamp antar trajectory (nearest timestamp pairing)."""
    pairs = []
    i, j = 0, 0
    while i < len(traj_ref) and j < len(traj_est):
        t_ref, t_est = traj_ref[i][0], traj_est[j][0]
        diff = t_ref - t_est
        if abs(diff) <= max_diff:
            pairs.append((i, j))
            i += 1
            j += 1
        elif diff > 0:
            j += 1
        else:
            i += 1
    ref_xyz = np.array([traj_ref[i][1] for i, _ in pairs])
    est_xyz = np.array([traj_est[j][1] for _, j in pairs])
    return ref_xyz, est_xyz

def main():
    parser = argparse.ArgumentParser(description="Hitung Absolute Trajectory Error (ATE) antara dua trajectory TUM.")
    parser.add_argument("ref", help="Trajectory groundtruth/reference (.txt)")
    parser.add_argument("est", help="Trajectory hasil estimasi (.txt)")
    parser.add_argument("--max-diff", type=float, default=0.02,
                        help="Toleransi waktu antar pose (detik)")
    args = parser.parse_args()

    traj_ref = load_tum_trajectory(args.ref)
    traj_est = load_tum_trajectory(args.est)
    ref_xyz, est_xyz = associate_trajectories(traj_ref, traj_est, max_diff=args.max_diff)

    print(f"Jumlah pasangan pose cocok: {len(ref_xyz)}")
    rmse, mae, errors = compute_ate(ref_xyz, est_xyz)

    print(f"\nATE RMSE : {rmse:.6f} m")
    print(f"ATE MAE  : {mae:.6f} m")
    print(f"Error min: {errors.min():.6f}  max: {errors.max():.6f}")

if __name__ == "__main__":
    main()
