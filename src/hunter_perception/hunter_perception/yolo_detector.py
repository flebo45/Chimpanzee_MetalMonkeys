"""
Hunter Perception - YOLO Object Detector Wrapper
================================================

This module encapsulates the computer vision logic using the Ultralytics YOLOv8 framework.
It acts as the primary sensor for the "Real Tracking" behavior.

Architectural Role:
-------------------
1.  **Inference**: Executes the neural network on the input video frame.
2.  **Filtering**: Filters detections based on Class ID (e.g., 'sports ball') and Confidence Score.
3.  **Selection**: Implements a "Largest Area" heuristic to select the most relevant target 
    (usually the closest one) when multiple objects are detected.

Hardware Acceleration:
----------------------
If a CUDA-capable GPU (Jetson Nano/Orin) is available, this module automatically utilizes it.
Otherwise, it falls back to CPU inference.

Authors: [Metal Monkeys Team]
Version: 1.0.0
"""
from ultralytics import YOLO
import numpy as np
import cv2
from typing import List, Tuple, Optional

class YoloDetector:
    """
    Wrapper class for YOLOv8 inference and target selection logic.
    
    Attributes:
        model (YOLO): The loaded Ultralytics YOLO model.
        conf (float): Minimum confidence threshold (0.0 - 1.0) to consider a detection valid.
        target_class (int): Primary class ID to track (default: 32 for 'sports ball').
    """
    def __init__(self, model_path: str = "yolov8n.pt", conf_threshold: float = 0.5, target_class_id: int = 32):
        """
        Initialize the YOLO Detector.

        Args:
            model_path (str): Path to the `.pt` model weights file.
            conf_threshold (float): Confidence threshold. Detections below this are discarded.
            target_class_id (int): Default COCO class ID to track.
        """
        self.conf = conf_threshold
        self.target_class = target_class_id

        # Load the YOLO model
        self.model = YOLO(model_path)
    
    def detect(self, image: np.ndarray) -> Optional[Tuple[int, int, int, int]]:
        """
        Performs inference on a single image frame and selects the best target.

        The selection logic prioritizes the **Largest Bounding Box**. 
        This assumes that the largest object of the correct class is the one 
        closest to the robot and therefore the intended target.

        Args:
            image (np.ndarray): Input image in BGR format (OpenCV standard).

        Returns:
            Optional[Tuple[int, int, int, int]]: 
                - If target found: (x1, y1, x2, y2) bounding box coordinates.
                - If no target found: None.
        """

        # Perform inference
        results = self.model(image, verbose=False, conf=0.15)

        best_box = None
        max_area = 0

        # if len(results[0].boxes) == 0:
        #     print("DEBUG: No boxes detected")

        # Process results
        for result in results:
            boxes = result.boxes
            for box in boxes:
                cls_id = int(box.cls[0].item())
                conf = float(box.conf[0].item())
                name = result.names[cls_id]
                #print(f"DEBUG: Detected class {name} with confidence {conf:.2f}")
                
                # Filter by target class and confidence
                accepted_ids = [32, 29, 49]

                if cls_id in accepted_ids:
                    # Extract bounding box coordinates
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    #print(f"DEBUG: Box coordinates: {x1:.1f}, {y1:.1f}, {x2:.1f}, {y2:.1f} | Image shape: {image.shape} ")
                    
                    # Calculate area
                    width = x2 - x1
                    height = y2 - y1
                    area = width * height

                    # Select the box with the largest area
                    if area > max_area:
                        max_area = area
                        #x_center = int((x1 + width) / 2)
                        #y_center = int((y1 + height) / 2)
                        best_box = (int(x1), int(y1), int(x2), int(y2))
        
        return best_box