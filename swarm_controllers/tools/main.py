import numpy as np
import open3d as o3d
import csv
import time
import os
 
def load_voxels_from_csv(filename):
    voxels = []
    if not os.path.exists(filename):
        return np.array(voxels)
    try:
        with open(filename, newline='') as csvfile:
            reader = csv.DictReader(csvfile)
            for row in reader:
                x = float(row['x'])
                y = float(row['y'])
                z = float(row['z'])
                voxels.append([x, y, z])
    except Exception as e:
        print(f"Error reading CSV: {e}")
    return np.array(voxels)
 
csv_file = "/home/traskos/ws_examples/src/swarm_controllers/tools/octomap_voxels.csv"
 
# Load initial points
voxels = load_voxels_from_csv(csv_file)
if voxels.size == 0:
    voxels = np.zeros((1, 3))  # dummy point so color works
 
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(voxels)
pcd.paint_uniform_color([0.2, 0.7, 0.3])  # green
 
vis = o3d.visualization.Visualizer()
vis.create_window("Realtime Octomap Viewer")
vis.add_geometry(pcd)
 
last_mtime = os.path.getmtime(csv_file) if os.path.exists(csv_file) else 0
 
try:
    while True:
        # Check if file updated
        if os.path.exists(csv_file):
            mtime = os.path.getmtime(csv_file)
            if mtime != last_mtime:
                last_mtime = mtime
                voxels = load_voxels_from_csv(csv_file)
                if voxels.size == 0:
                    voxels = np.zeros((1, 3))  # avoid empty point cloud
                pcd.points = o3d.utility.Vector3dVector(voxels)
                pcd.paint_uniform_color([0.2, 0.7, 0.3])
                vis.update_geometry(pcd)
                vis.poll_events()
                vis.update_renderer()
                print(f"Updated point cloud with {len(voxels)} voxels")
 
        vis.poll_events()
        vis.update_renderer()
        time.sleep(0.1)
 
except KeyboardInterrupt:
    print("Closing visualization...")
finally:
    vis.destroy_window()
 