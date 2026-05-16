from abc import ABC, abstractmethod
import cv2
import numpy as np

class ObjectDetector(ABC):
    """Base class for any detection model (Color, YOLO, etc.)"""
    @abstractmethod
    def detect(self, frame):
        """Returns list of (x, y, w, h, label)"""
        pass

class YoloDetector(ObjectDetector):
    """Detects objects using a trained YOLOv8 model."""
    def __init__(self, model_path="models/yolov8n.pt", confidence=0.5):
        try:
            from ultralytics import YOLO
            self.model = YOLO(model_path)
        except ImportError:
            print("-- [VISION] Error: ultralytics package not found. YOLO disabled.")
            self.model = None
        self.conf = confidence

    def detect(self, frame):
        results_list = []
        if self.model is None or frame is None:
            return results_list

        results = self.model(frame, verbose=False, conf=self.conf)
        for result in results:
            boxes = result.boxes
            for box in boxes:
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                w = int(x2 - x1)
                h = int(y2 - y1)
                cls_id = int(box.cls[0].cpu().numpy())
                label = self.model.names[cls_id]
                results_list.append((int(x1), int(y1), w, h, label))
                
        return results_list

class ColorDetector(ObjectDetector):
    """Detects objects based on HSV Color"""
    def __init__(self, color_name="red"):
        color = color_name.lower()
        
        self.colors = {
            "red": [
                (np.array([0, 50, 50]), np.array([10, 255, 255])),
                (np.array([170, 50, 50]), np.array([180, 255, 255]))
            ],
            "white": [
                (np.array([0, 0, 200]), np.array([180, 60, 255]))
            ]
        }
        
        if color not in self.colors:
            print(f"-- [VISION] Warning: Color '{color}' not found, defaulting to RED")
            self.ranges = self.colors["red"]
        else:
            self.ranges = self.colors[color]
            
    def detect(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = np.zeros(hsv.shape[:2], dtype="uint8")
        
        for (lower, upper) in self.ranges:
            mask += cv2.inRange(hsv, lower, upper)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        results = []
        for cnt in contours:
            if cv2.contourArea(cnt) > 500:
                x, y, w, h = cv2.boundingRect(cnt)
                results.append((x, y, w, h, "ColorTarget"))
        return results
