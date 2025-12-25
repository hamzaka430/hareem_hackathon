---
sidebar_position: 2
---

# Object Detection

Object detection enables robots to identify and locate objects in their environment. This section covers classical and deep learning approaches.

## Classical Object Detection

### Template Matching

```python
import cv2
import numpy as np

def template_match(image, template):
    result = cv2.matchTemplate(image, template, cv2.TM_CCOEFF_NORMED)
    min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(result)
    
    # Get bounding box
    h, w = template.shape[:2]
    top_left = max_loc
    bottom_right = (top_left[0] + w, top_left[1] + h)
    
    return top_left, bottom_right, max_val
```

**Limitations**: Scale and rotation sensitive

### Contour-Based Detection

```python
def detect_shapes(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    _, thresh = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)
    
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    detections = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > 100:  # Filter small contours
            x, y, w, h = cv2.boundingRect(contour)
            detections.append({'bbox': (x, y, w, h), 'area': area})
    
    return detections
```

### Color-Based Detection

```python
def detect_by_color(image, lower_hsv, upper_hsv):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_hsv, upper_hsv)
    
    # Clean up mask
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    
    # Find contours
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    return contours, mask
```

## Deep Learning Object Detection

### Overview of Architectures

| Architecture | Speed | Accuracy | Use Case |
|-------------|-------|----------|----------|
| YOLO (v8) | Very Fast | Good | Real-time |
| SSD | Fast | Good | Mobile |
| Faster R-CNN | Slow | Very High | Accuracy critical |
| RT-DETR | Fast | Very High | Modern real-time |

### YOLOv8 with Ultralytics

```python
from ultralytics import YOLO

# Load model
model = YOLO('yolov8n.pt')  # nano model

# Run inference
results = model(image)

# Process results
for result in results:
    boxes = result.boxes
    for box in boxes:
        # Bounding box
        x1, y1, x2, y2 = box.xyxy[0].numpy()
        
        # Confidence
        confidence = box.conf[0].numpy()
        
        # Class
        class_id = int(box.cls[0].numpy())
        class_name = model.names[class_id]
        
        print(f'{class_name}: {confidence:.2f} at ({x1}, {y1}, {x2}, {y2})')
```

### Training Custom Models

```python
from ultralytics import YOLO

# Create dataset structure:
# dataset/
# ├── train/
# │   ├── images/
# │   └── labels/
# └── val/
#     ├── images/
#     └── labels/

# dataset.yaml
"""
path: /path/to/dataset
train: train/images
val: val/images

names:
  0: class1
  1: class2
"""

# Train model
model = YOLO('yolov8n.pt')
model.train(data='dataset.yaml', epochs=100, imgsz=640)
```

### ONNX Deployment

```python
# Export to ONNX
model.export(format='onnx')

# Load ONNX model
import onnxruntime as ort

session = ort.InferenceSession('yolov8n.onnx')

# Run inference
outputs = session.run(None, {'images': input_tensor})
```

## Pose Estimation

### 2D Keypoint Detection

```python
from ultralytics import YOLO

# Load pose model
model = YOLO('yolov8n-pose.pt')

results = model(image)

for result in results:
    keypoints = result.keypoints
    for kp in keypoints:
        xy = kp.xy[0].numpy()  # (17, 2) keypoints
        conf = kp.conf[0].numpy()  # Confidence scores
```

**COCO Keypoints**:
```
0: nose, 1: left_eye, 2: right_eye, 3: left_ear, 4: right_ear,
5: left_shoulder, 6: right_shoulder, 7: left_elbow, 8: right_elbow,
9: left_wrist, 10: right_wrist, 11: left_hip, 12: right_hip,
13: left_knee, 14: right_knee, 15: left_ankle, 16: right_ankle
```

### 6-DoF Object Pose

For manipulation, we need full 3D pose:

```python
# Using PoseCNN or similar
# Output: Translation (x, y, z) + Rotation (quaternion or matrix)

pose = {
    'translation': [0.5, 0.2, 0.8],  # meters
    'rotation': [0, 0, 0, 1],  # quaternion (x, y, z, w)
}
```

## Instance Segmentation

Beyond bounding boxes—pixel-level masks.

```python
from ultralytics import YOLO

# Load segmentation model
model = YOLO('yolov8n-seg.pt')

results = model(image)

for result in results:
    if result.masks is not None:
        masks = result.masks.data.numpy()  # Binary masks
        
        for i, mask in enumerate(masks):
            # mask is a binary array
            # Apply to image for visualization
            colored_mask = np.zeros_like(image)
            colored_mask[mask > 0.5] = [0, 255, 0]
```

## ROS2 Integration

### Detection Node

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge
from ultralytics import YOLO

class ObjectDetector(Node):
    def __init__(self):
        super().__init__('object_detector')
        
        self.bridge = CvBridge()
        self.model = YOLO('yolov8n.pt')
        
        self.image_sub = self.create_subscription(
            Image, 'camera/image_raw', self.detect_callback, 10)
        
        self.detection_pub = self.create_publisher(
            Detection2DArray, 'detections', 10)
        
        self.get_logger().info('Object Detector initialized')
    
    def detect_callback(self, msg):
        # Convert to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        
        # Run detection
        results = self.model(cv_image, verbose=False)
        
        # Create detection message
        detection_array = Detection2DArray()
        detection_array.header = msg.header
        
        for result in results:
            for box in result.boxes:
                detection = Detection2D()
                
                # Bounding box
                x1, y1, x2, y2 = box.xyxy[0].numpy()
                detection.bbox.center.position.x = (x1 + x2) / 2
                detection.bbox.center.position.y = (y1 + y2) / 2
                detection.bbox.size_x = float(x2 - x1)
                detection.bbox.size_y = float(y2 - y1)
                
                # Class and score
                hypothesis = ObjectHypothesisWithPose()
                hypothesis.hypothesis.class_id = self.model.names[int(box.cls[0])]
                hypothesis.hypothesis.score = float(box.conf[0])
                detection.results.append(hypothesis)
                
                detection_array.detections.append(detection)
        
        self.detection_pub.publish(detection_array)

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

## Performance Optimization

### GPU Acceleration

```python
import torch

# Check GPU availability
print(f'CUDA available: {torch.cuda.is_available()}')

# YOLO automatically uses GPU if available
model = YOLO('yolov8n.pt')
# Force GPU
model.to('cuda')
```

### TensorRT Optimization

```python
# Export to TensorRT (NVIDIA GPUs)
model.export(format='engine', device=0)

# Load optimized model
model = YOLO('yolov8n.engine')
```

### Batch Processing

```python
# Process multiple images at once
images = [img1, img2, img3, img4]
results = model(images, batch=4)
```

## Key Takeaways

1. **Classical methods** work for simple, controlled scenarios
2. **YOLO** is the go-to for real-time object detection
3. **Instance segmentation** provides pixel-level understanding
4. **Pose estimation** is critical for manipulation
5. **GPU acceleration** is essential for real-time performance

---

*Next: [Depth Perception](./depth-perception.md) →*
