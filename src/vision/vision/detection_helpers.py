import cv2
import numpy as np

from vision.utils import centerOfMass, orientationVectors, points_inside_contour


def largest_contour(contours, min_area=0.0):
    largest = None
    largest_area = min_area
    for contour in contours:
        contour_area = cv2.contourArea(contour)
        if contour_area < min_area:
            continue
        if largest is None or contour_area > largest_area:
            largest = contour
            largest_area = contour_area
    return largest


def detect_mounting_dock(hsv):
    lower_green = np.array([35, 50, 40], dtype=np.uint8)
    upper_green = np.array([90, 255, 255], dtype=np.uint8)
    green_mask = cv2.inRange(hsv, lower_green, upper_green)

    kernel = np.ones((5, 5), np.uint8)
    green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_OPEN, kernel, iterations=1)
    green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_CLOSE, kernel, iterations=2)

    object_contours, _ = cv2.findContours(
        green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    obj = largest_contour(object_contours, min_area=500.0)
    if obj is None:
        return green_mask, None, None
    return green_mask, obj, cv2.boundingRect(obj)


def detect_robotic_arm(hsv, image_shape):
    lower_black = np.array([0, 0, 0], dtype=np.uint8)
    upper_black = np.array([200, 255, 100], dtype=np.uint8)
    black_mask = cv2.inRange(hsv, lower_black, upper_black)

    kernel = np.ones((2, 2), np.uint8)
    black_mask = cv2.morphologyEx(black_mask, cv2.MORPH_OPEN, kernel, iterations=1)
    black_mask = cv2.morphologyEx(black_mask, cv2.MORPH_CLOSE, kernel, iterations=2)

    black_contours, _ = cv2.findContours(
        black_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )

    merged_mask = np.zeros(image_shape[:2], dtype=np.uint8)
    cv2.drawContours(merged_mask, black_contours, -1, 255, thickness=cv2.FILLED)
    union_black_contours, _ = cv2.findContours(
        merged_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )

    filtered_contours = [
        contour for contour in union_black_contours if cv2.contourArea(contour) >= 300.0
    ]
    max_black_contour = largest_contour(filtered_contours, min_area=300.0)
    return filtered_contours, max_black_contour


def compute_arm_pose(contour, image_shape):
    if contour is None or len(contour) < 5:
        return None

    black_pts, _ = points_inside_contour(contour, image_shape[:2])
    if len(black_pts) == 0:
        return None

    ellipse = cv2.fitEllipse(contour)
    (cx, cy), (w, h), angle_deg = ellipse

    theta = np.deg2rad(angle_deg)
    ux, uy = np.cos(theta), np.sin(theta)
    if h > w:
        ux, uy = -np.sin(theta), np.cos(theta)

    shift = 0.45 * max(w, h)
    new_cx = cx - shift * ux
    new_cy = cy - shift * uy

    maybe_inside = cv2.pointPolygonTest(
        contour, (new_cx - 2 * shift * ux, new_cy - 2 * shift * uy), False
    ) >= 0
    if maybe_inside:
        new_cx = cx + shift * ux
        new_cy = cy + shift * uy

    center = (int(new_cx), int(new_cy))
    axis_a, axis_b = orientationVectors(black_pts)
    return {
        'center': center,
        'axis_a': tuple((np.array(center) + axis_a).tolist()),
        'axis_b': tuple((np.array(center) + axis_b).tolist()),
    }


def detect_white_model(hsv, bbox, frame_shape, arm_center, min_white_model_area):
    white_mask = cv2.inRange(
        hsv,
        np.array([0, 0, 170], dtype=np.uint8),
        np.array([180, 80, 255], dtype=np.uint8),
    )
    white_mask = cv2.morphologyEx(
        white_mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8), iterations=1
    )
    white_mask = cv2.morphologyEx(
        white_mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8), iterations=2
    )

    x, y, w, h = bbox
    roi_margin = 20
    x0 = max(0, x - roi_margin)
    y0 = max(0, y - roi_margin)
    x1 = min(frame_shape[1], x + w + roi_margin)
    y1 = min(frame_shape[0], y + h + roi_margin)

    white_roi_mask = np.zeros_like(white_mask)
    white_roi_mask[y0:y1, x0:x1] = 255
    white_mask = cv2.bitwise_and(white_mask, white_roi_mask)

    white_contours, _ = cv2.findContours(
        white_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    max_white_contour = largest_contour(
        white_contours, min_area=min_white_model_area
    )
    if max_white_contour is None:
        return None

    white_center = tuple(centerOfMass(max_white_contour).tolist())
    white_dx = float(arm_center[0] - white_center[0])
    white_dy = float(arm_center[1] - white_center[1])
    return {
        'contour': max_white_contour,
        'center': white_center,
        'distance': float(np.hypot(white_dx, white_dy)),
    }


def detect_holes(green_mask, obj_mask, arm_center, min_hole_area):
    holes_mask = cv2.bitwise_and(cv2.bitwise_not(green_mask), obj_mask)
    holes_mask = cv2.morphologyEx(
        holes_mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8)
    )

    hole_contours, _ = cv2.findContours(
        holes_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    filtered_holes = [
        contour for contour in hole_contours if cv2.contourArea(contour) >= min_hole_area
    ]

    max_hole_contour = largest_contour(filtered_holes, min_area=min_hole_area)
    if max_hole_contour is None:
        return holes_mask, filtered_holes, None

    hole_center = tuple(np.mean(max_hole_contour.reshape(-1, 2), axis=0).astype(int))
    dx = float(arm_center[0] - hole_center[0])
    dy = float(arm_center[1] - hole_center[1])
    return holes_mask, filtered_holes, {
        'center': hole_center,
        'dx': dx,
        'dy': dy,
        'distance': float(np.hypot(dx, dy)),
    }
