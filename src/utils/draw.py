"""
draw.py
--------
Visualization utilities for the interceptor demo.

Provides simple OpenCV-based drawing helpers for:
- Displaying bounding boxes from detection/tracking.
- Overlaying textual information (HUD) such as confidence, range, and mode.

These utilities are used in the live display window (`main.py --show`)
to help debug perception and control modules in real time.
"""

import cv2
import numpy as np


# ---------------------------------------------------------------------------
# Draw bounding box
# ---------------------------------------------------------------------------
def draw_bbox(frame, xywh, color=(0, 255, 0), label=None):
    """
    Draw a bounding box and optional label on an image frame.

    Parameters
    ----------
    frame : np.ndarray
        The image (BGR, as used by OpenCV).
    xywh : tuple or np.ndarray
        Bounding box in (center_x, center_y, width, height) pixel format.
        - Typically output of YOLO or the tracker.
    color : tuple(int, int, int), optional
        Box color in BGR format. Default = green (0,255,0).
    label : str, optional
        Text label to display above the box (e.g., confidence, "pred").

    Returns
    -------
    np.ndarray
        The same frame with the bounding box drawn on it.

    Notes
    -----
    - Uses OpenCV’s `cv2.rectangle` and `cv2.putText`.
    - Automatically clips text position if it would go above the frame.
    """
    if xywh is None:
        # Nothing to draw
        return frame

    # Unpack YOLO-style bounding box center format → top-left/bottom-right
    x, y, w, h = xywh
    x1 = int(x - w / 2)
    y1 = int(y - h / 2)
    x2 = int(x + w / 2)
    y2 = int(y + h / 2)

    # Draw rectangle
    cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)

    # Optional label text just above top-left corner
    if label:
        cv2.putText(
            frame,
            label,
            (x1, max(0, y1 - 5)),  # avoid going above image
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,  # font scale
            color,
            1,    # thickness
        )

    return frame


# ---------------------------------------------------------------------------
# Draw on-screen HUD (heads-up display)
# ---------------------------------------------------------------------------
def put_hud(frame, lines):
    """
    Overlay a simple text-based heads-up display (HUD) on the image.

    Parameters
    ----------
    frame : np.ndarray
        The image to draw on.
    lines : list[str]
        A list of strings, each rendered on a new line.

    Returns
    -------
    np.ndarray
        The same frame with text annotations.

    Notes
    -----
    - Used in `main.py` to show mode, confidence, range, and velocity commands.
    - Text is white and left-aligned at a fixed screen offset.
    """
    y = 18  # initial vertical offset
    for text in lines:
        cv2.putText(
            frame,
            text,
            (8, y),  # (x, y) position
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,               # font size
            (255, 255, 255),    # color: white
            1,                  # thickness
        )
        y += 18  # line spacing
    return frame
