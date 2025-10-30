import open3d as o3d
import numpy as np

def compute_rmse_pointcloud(pcd1_path, pcd2_path):
    pcd1 = o3d.io.read_point_cloud(pcd1_path)
    pcd2 = o3d.io.read_point_cloud(pcd2_path)

    # Pastikan jumlah titik cukup dan cloud tidak kosong
    if len(pcd1.points) == 0 or len(pcd2.points) == 0:
        raise ValueError("Salah satu point cloud kosong.")

    # Registrasi kasar (opsional, kalau belum align)
    threshold = 0.05  # 5 cm
    trans_init = np.identity(4)
    reg = o3d.pipelines.registration.registration_icp(
        pcd2, pcd1, threshold, trans_init,
        o3d.pipelines.registration.TransformationEstimationPointToPoint()
    )
    pcd2_aligned = pcd2.transform(reg.transformation)

    # Hitung jarak per titik terdekat
    dists = pcd1.compute_point_cloud_distance(pcd2_aligned)
    rmse = np.sqrt(np.mean(np.square(dists)))
    mae = np.mean(np.abs(dists))

    print(f"RMSE: {rmse:.6f} m")
    print(f"MAE : {mae:.6f} m")

compute_rmse_pointcloud("map_ref.ply", "map_test.ply")
