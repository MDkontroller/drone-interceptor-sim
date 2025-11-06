"""
detector.py
------------
Perception module responsible for detecting drones (or generic targets)
in camera frames using a YOLOv8 model from Ultralytics.

This acts as the first step in the perception pipeline:
- Takes a BGR frame (from OpenCV).
- Runs object detection using YOLOv8.
- Returns the highest-confidence bounding box and its confidence score.

Notes
-----
• Model: Default is 'yolov8n.pt' (nano), which is small and fast enough
  for real-time inference (10–20 Hz on GPU). You can swap it for
  'yolov8s.pt' or a custom fine-tuned model for better accuracy.
• Input format: The Ultralytics API supports RGB or BGR images. We use
  BGR directly for convenience (as OpenCV provides BGR frames).
• Output format: Bounding boxes are (center_x, center_y, width, height)
  in pixel coordinates.

Usage Example
-------------
>>> from perception.detector import YoloDroneDetector
>>> det = YoloDroneDetector("yolov8n.pt")
>>> xywh, conf = det.detect(frame_bgr)
>>> if conf > 0.5:
>>>     print("Target detected at:", xywh)
"""

from ultralytics import YOLO
import numpy as np
import config as C


class YoloDroneDetector:
    """
    YOLO-based drone detector wrapper around Ultralytics API.

    Methods
    -------
    detect(frame_bgr):
        Runs YOLO inference on a single BGR frame and returns the
        highest-confidence bounding box (xywh) and its confidence score.
    """

    def __init__(self, model_name: str = "yolov8n.pt"):
        """
        Parameters
        ----------
        model_name : str, optional
            Name or path of the YOLOv8 model to load.
            Defaults to "yolov8n.pt" (smallest and fastest).
        """
        self.model = YOLO(model_name)

    def detect(self, frame_bgr):
        """
        Run object detection on a frame and return the best detection.

        Parameters
        ----------
        frame_bgr : np.ndarray
            Input frame from OpenCV in BGR format, shape (H, W, 3).

        Returns
        -------
        (xywh, conf) : (np.ndarray(4,), float)
            - xywh: Bounding box (center_x, center_y, width, height) in pixels.
            - conf: Confidence score (0.0–1.0) for that detection.
          If no detection is found, returns (None, 0.0).

        Notes
        -----
        - Uses the config thresholds:
          * `C.DET_CONF_THRES`: minimum confidence
          * `C.DET_IOU_THRES`: IoU threshold for NMS
        - Selects the **highest-confidence** bounding box among all detections.
        """
        # Run inference (suppress verbose logging for cleaner output)
        results = self.model.predict(
            frame_bgr,
            imgsz=480,
            conf=C.DET_CONF_THRES,
            iou=C.DET_IOU_THRES,
            verbose=False,
        )

        boxes = results[0].boxes
        if boxes is None or boxes.xywh is None or len(boxes.xywh) == 0:
            # No detections → return defaults
            return None, 0.0

        # Extract bounding boxes and confidences as numpy arrays
        xywh = boxes.xywh.cpu().numpy()  # (N, 4)
        conf = boxes.conf.cpu().numpy()  # (N,)

        # Pick the most confident detection
        idx = int(np.argmax(conf))
        return xywh[idx], float(conf[idx])

