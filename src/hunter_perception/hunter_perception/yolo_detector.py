from ultralytics import YOLO
import numpy as np
import cv2
from typing import List, Tuple, Optional

class YoloDetector:
    def __init__(self, model_path: str = "yolov8n.pt", conf_threshold: float = 0.5, target_class_id: int = 32):
        """
        Initialize YOLO Detector
        Args:
            model_path (str): Path to the YOLO model weights
            conf_threshold (float): Confidence threshold for detections
            target_class_id (int): Class ID to filter detections (default is 32 for 'sports ball')
        """
        self.conf = conf_threshold
        self.target_class = target_class_id

        self.model = YOLO(model_path)
    
    def detect(self, image: np.ndarray) -> Optional[Tuple[int, int, int, int]]:
        """
        Infer on a frame
        Args:
            image (np.ndarray): Input image in BGR format
        Returns:
            Tuple (x_center, y_center, width, height) of the detected object or None if not found
        """

        # Perform inference
        results = self.model(image, verbose=False, conf=0.15)

        best_box = None
        max_area = 0

        # if len(results[0].boxes) == 0:
        #     print("DEBUG: No boxes detected")

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
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    #print(f"DEBUG: Box coordinates: {x1:.1f}, {y1:.1f}, {x2:.1f}, {y2:.1f} | Image shape: {image.shape} ")
                    width = x2 - x1
                    height = y2 - y1
                    area = width * height

                    if area > max_area:
                        max_area = area
                        #x_center = int((x1 + width) / 2)
                        #y_center = int((y1 + height) / 2)
                        best_box = (int(x1), int(y1), int(x2), int(y2))
        
        return best_box