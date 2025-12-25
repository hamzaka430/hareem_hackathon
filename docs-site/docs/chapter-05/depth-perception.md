---
sidebar_position: 3
---

# Depth Perception

Depth perception allows robots to understand the 3D structure of their environment. This section covers depth sensing technologies and processing techniques.

## Depth Sensing Technologies

### Stereo Vision

Uses two cameras to triangulate depth:

```
Left Camera          Right Camera
    │                    │
    │    Object          │
    │      ●             │
    │     /|\            │
    │    / | \           │
    ▼   /  |  \          ▼
   [L] /   |   \ [R]
      │    d    │
      │◄──────►│
       baseline
```

**Depth formula**:

$$
Z = \frac{f \cdot B}{d}
$$

Where:
- $Z$: Depth
- $f$: Focal length
- $B$: Baseline (distance between cameras)
- $d$: Disparity (pixel difference)

### Structured Light

Projects known pattern and measures deformation:

```python
# Intel RealSense D435 example
import pyrealsense2 as rs

# Configure pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)

# Get frames
frames = pipeline.wait_for_frames()
depth_frame = frames.get_depth_frame()
color_frame = frames.get_color_frame()

# Convert to numpy
depth_image = np.asanyarray(depth_frame.get_data())
color_image = np.asanyarray(color_frame.get_data())
```

### Time-of-Flight (ToF)

Measures light travel time:

```
Camera ─── Pulse ───▶ Object
       ◀── Reflection ───
       
       Time = 2d/c
       d = c × Time / 2
```

## Point Clouds

### Understanding Point Clouds

A point cloud is a set of 3D points:

```python
import numpy as np

# Point cloud: N x 3 (or N x 6 with RGB)
points = np.array([
    [x1, y1, z1],
    [x2, y2, z2],
    # ...
])

# With color
points_rgb = np.array([
    [x1, y1, z1, r1, g1, b1],
    [x2, y2, z2, r2, g2, b2],
    # ...
])
```

### Generating Point Clouds from Depth

```python
import numpy as np

def depth_to_pointcloud(depth_image, intrinsics):
    """Convert depth image to point cloud."""
    fx, fy = intrinsics['fx'], intrinsics['fy']
    cx, cy = intrinsics['cx'], intrinsics['cy']
    
    height, width = depth_image.shape
    
    # Create pixel coordinates
    u = np.arange(width)
    v = np.arange(height)
    u, v = np.meshgrid(u, v)
    
    # Convert to 3D
    z = depth_image / 1000.0  # mm to meters
    x = (u - cx) * z / fx
    y = (v - cy) * z / fy
    
    # Stack to point cloud
    points = np.stack([x, y, z], axis=-1)
    
    # Remove invalid points
    valid = z > 0
    points = points[valid]
    
    return points
```

### Point Cloud Libraries

**Open3D** (recommended):
```python
import open3d as o3d

# Create point cloud
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(points)
pcd.colors = o3d.utility.Vector3dVector(colors / 255.0)

# Visualize
o3d.visualization.draw_geometries([pcd])

# Save/Load
o3d.io.write_point_cloud("cloud.ply", pcd)
pcd = o3d.io.read_point_cloud("cloud.ply")
```

## Point Cloud Processing

### Filtering

```python
import open3d as o3d

# Statistical outlier removal
pcd_clean, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)

# Radius outlier removal
pcd_clean, _ = pcd.remove_radius_outlier(nb_points=16, radius=0.05)

# Voxel downsampling
pcd_down = pcd.voxel_down_sample(voxel_size=0.01)  # 1cm voxels
```

### Normal Estimation

```python
# Estimate normals
pcd.estimate_normals(
    search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))

# Orient normals consistently
pcd.orient_normals_consistent_tangent_plane(k=15)
```

### Plane Segmentation (RANSAC)

```python
# Segment the largest plane (e.g., table, floor)
plane_model, inliers = pcd.segment_plane(
    distance_threshold=0.01,
    ransac_n=3,
    num_iterations=1000
)

# Extract plane and remaining points
plane_cloud = pcd.select_by_index(inliers)
remaining_cloud = pcd.select_by_index(inliers, invert=True)

# Plane equation: ax + by + cz + d = 0
a, b, c, d = plane_model
```

### Clustering

```python
# DBSCAN clustering
labels = np.array(pcd.cluster_dbscan(eps=0.02, min_points=10))

# Get unique clusters
unique_labels = set(labels)
n_clusters = len(unique_labels) - (1 if -1 in labels else 0)

# Color clusters
max_label = labels.max()
colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])
```

## Registration (Alignment)

### ICP (Iterative Closest Point)

```python
# Point-to-point ICP
threshold = 0.02
trans_init = np.eye(4)  # Initial transformation

reg_result = o3d.pipelines.registration.registration_icp(
    source_pcd, target_pcd, threshold, trans_init,
    o3d.pipelines.registration.TransformationEstimationPointToPoint())

# Apply transformation
source_pcd.transform(reg_result.transformation)

print(f"Fitness: {reg_result.fitness}")
print(f"RMSE: {reg_result.inlier_rmse}")
```

### Global Registration

For large initial misalignment:

```python
# FPFH feature extraction
def compute_fpfh(pcd, voxel_size):
    pcd_down = pcd.voxel_down_sample(voxel_size)
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2, max_nn=30))
    
    fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 5, max_nn=100))
    
    return pcd_down, fpfh

# RANSAC registration
result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
    source_down, target_down, source_fpfh, target_fpfh,
    mutual_filter=True,
    max_correspondence_distance=voxel_size * 1.5,
    estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(),
    ransac_n=3,
    checkers=[],
    criteria=o3d.pipelines.registration.RANSACConvergenceCriteria(100000, 0.999))
```

## ROS2 Integration

### Point Cloud Messages

```python
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

class PointCloudProcessor(Node):
    def __init__(self):
        super().__init__('pointcloud_processor')
        
        self.subscription = self.create_subscription(
            PointCloud2, 'camera/depth/points', self.callback, 10)
        
        self.publisher = self.create_publisher(
            PointCloud2, 'processed_points', 10)
    
    def callback(self, msg):
        # Convert to numpy
        points = []
        for p in pc2.read_points(msg, field_names=['x', 'y', 'z']):
            points.append([p[0], p[1], p[2]])
        points = np.array(points)
        
        # Process with Open3D
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        
        # Filter
        pcd_filtered, _ = pcd.remove_statistical_outlier(20, 2.0)
        
        # Downsample
        pcd_down = pcd_filtered.voxel_down_sample(0.01)
        
        # Convert back to ROS message
        # ... publish processed cloud
```

## 3D Object Detection

### PointNet / PointNet++

Deep learning directly on point clouds:

```python
# Using mmdetection3d or similar
# Input: Point cloud (N x 3 or N x 4 with intensity)
# Output: 3D bounding boxes

# Pseudo-code
model = PointPillars()
boxes_3d = model.detect(point_cloud)

# Box format: [x, y, z, l, w, h, yaw]
for box in boxes_3d:
    center = box[:3]
    dimensions = box[3:6]
    rotation = box[6]
```

## Key Takeaways

1. **Multiple sensing modalities** (stereo, structured light, ToF) have trade-offs
2. **Point clouds** represent 3D environments as sets of points
3. **Open3D** is excellent for point cloud processing
4. **Filtering and segmentation** extract meaningful structure
5. **Registration** aligns multiple point clouds
6. **Deep learning** enables 3D object detection

---

*Next Chapter: [Motion Planning & Control](../chapter-06/motion-planning.md) →*
