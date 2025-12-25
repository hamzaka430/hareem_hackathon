---
sidebar_position: 1
---

# Vision Fundamentals

Computer vision enables robots to perceive and understand their environment through cameras. This chapter covers the essential concepts for robotic vision systems.

## Why Vision for Robotics?

Vision provides rich environmental information:
- Object recognition and localization
- Navigation and mapping
- Human detection and tracking
- Quality inspection
- Manipulation guidance

## Camera Types

### RGB Cameras

Standard color cameras providing 2D images.

| Specification | Description | Typical Range |
|--------------|-------------|---------------|
| Resolution | Pixels (width × height) | 640×480 to 4K |
| Frame rate | Images per second | 30-120 fps |
| Field of view | Angular coverage | 60°-180° |
| Interface | Data connection | USB, MIPI, GigE |

### Depth Cameras

Provide per-pixel distance measurements.

**Technologies**:

| Technology | Principle | Range | Pros | Cons |
|------------|-----------|-------|------|------|
| Structured Light | Pattern projection | 0.3-5m | High accuracy | Fails outdoors |
| Time-of-Flight (ToF) | Light travel time | 0.5-10m | Fast, compact | Lower resolution |
| Stereo | Triangulation | 0.5-20m | Passive, cheap | Needs texture |

**Popular Sensors**:
- Intel RealSense D435i (stereo + IMU)
- Microsoft Azure Kinect (ToF)
- ZED 2 (stereo, long range)

### Event Cameras

Asynchronous sensors that detect brightness changes.

```
Traditional Camera:     Event Camera:
Frame 1 → Frame 2      Events as they occur
   │         │         ● ● ●
   │  30ms   │         ●   ●●
   ↓         ↓           ●●
[Image]   [Image]      Microsecond resolution
```

## Image Representation

### Digital Images

An image is a 2D array of pixels:

```python
import numpy as np

# Grayscale image: H × W
gray_image = np.zeros((480, 640), dtype=np.uint8)

# Color image: H × W × 3 (BGR in OpenCV)
color_image = np.zeros((480, 640, 3), dtype=np.uint8)

# Depth image: H × W (usually uint16 or float32)
depth_image = np.zeros((480, 640), dtype=np.uint16)
```

### Color Spaces

| Space | Channels | Use Case |
|-------|----------|----------|
| RGB/BGR | Red, Green, Blue | Display, storage |
| HSV | Hue, Saturation, Value | Color segmentation |
| Grayscale | Intensity | Edge detection |
| YUV | Luminance, Chrominance | Compression |

### Coordinate Systems

```
Image Coordinates        Camera Coordinates
┌───────────→ u         
│ (0,0)                       Y↑  Z (optical axis)
│                              │ /
│    ● (u,v)                   │/
↓                              └────→ X
v                            Camera center
```

## Camera Calibration

### Intrinsic Parameters

The camera matrix $K$:

$$
K = \begin{bmatrix}
f_x & 0 & c_x \\
0 & f_y & c_y \\
0 & 0 & 1
\end{bmatrix}
$$

Where:
- $f_x, f_y$: Focal lengths in pixels
- $c_x, c_y$: Principal point (image center)

### Distortion Parameters

Lens distortion coefficients $(k_1, k_2, p_1, p_2, k_3)$:
- $k_n$: Radial distortion
- $p_n$: Tangential distortion

### Calibration Process

```python
import cv2
import numpy as np

# Prepare object points (checkerboard corners)
objp = np.zeros((6*9, 3), np.float32)
objp[:,:2] = np.mgrid[0:9, 0:6].T.reshape(-1, 2)

# Collect images of checkerboard
objpoints = []  # 3D points
imgpoints = []  # 2D points

for image in calibration_images:
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(gray, (9, 6), None)
    
    if ret:
        objpoints.append(objp)
        imgpoints.append(corners)

# Calibrate
ret, K, dist, rvecs, tvecs = cv2.calibrateCamera(
    objpoints, imgpoints, gray.shape[::-1], None, None)
```

## Image Processing Fundamentals

### Filtering

```python
import cv2

# Gaussian blur (noise reduction)
blurred = cv2.GaussianBlur(image, (5, 5), 0)

# Bilateral filter (edge-preserving)
bilateral = cv2.bilateralFilter(image, 9, 75, 75)

# Median filter (salt & pepper noise)
median = cv2.medianBlur(image, 5)
```

### Edge Detection

```python
# Canny edge detector
edges = cv2.Canny(gray_image, 50, 150)

# Sobel gradients
sobel_x = cv2.Sobel(gray_image, cv2.CV_64F, 1, 0, ksize=3)
sobel_y = cv2.Sobel(gray_image, cv2.CV_64F, 0, 1, ksize=3)
```

### Morphological Operations

```python
kernel = np.ones((5, 5), np.uint8)

# Erosion - shrink white regions
eroded = cv2.erode(binary_image, kernel, iterations=1)

# Dilation - expand white regions
dilated = cv2.dilate(binary_image, kernel, iterations=1)

# Opening - erosion then dilation (remove noise)
opened = cv2.morphologyEx(binary_image, cv2.MORPH_OPEN, kernel)

# Closing - dilation then erosion (fill holes)
closed = cv2.morphologyEx(binary_image, cv2.MORPH_CLOSE, kernel)
```

## Feature Detection

### Interest Points

**Harris Corners**:
```python
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
corners = cv2.cornerHarris(gray, 2, 3, 0.04)
```

**ORB Features** (good for real-time):
```python
orb = cv2.ORB_create()
keypoints, descriptors = orb.detectAndCompute(image, None)
```

**SIFT Features** (more robust):
```python
sift = cv2.SIFT_create()
keypoints, descriptors = sift.detectAndCompute(image, None)
```

### Feature Matching

```python
# Create matcher
bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

# Match descriptors
matches = bf.match(descriptors1, descriptors2)

# Sort by distance
matches = sorted(matches, key=lambda x: x.distance)
```

## Integration with ROS2

### Publishing Images

```python
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        self.publisher = self.create_publisher(Image, 'camera/image_raw', 10)
        self.bridge = CvBridge()
        self.timer = self.create_timer(0.033, self.publish_frame)  # 30 Hz
        self.cap = cv2.VideoCapture(0)
    
    def publish_frame(self):
        ret, frame = self.cap.read()
        if ret:
            msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            self.publisher.publish(msg)
```

### Processing Images

```python
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ImageProcessor(Node):
    def __init__(self):
        super().__init__('image_processor')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image, 'camera/image_raw', self.process_callback, 10)
    
    def process_callback(self, msg):
        # Convert to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        
        # Process...
        processed = cv2.Canny(cv_image, 50, 150)
```

## Key Takeaways

1. **Camera types** serve different purposes (RGB, depth, event)
2. **Calibration** is essential for accurate measurements
3. **Image processing** enables feature extraction
4. **OpenCV** is the standard library for computer vision
5. **cv_bridge** connects OpenCV and ROS2

---

*Next: [Object Detection](./object-detection.md) →*
