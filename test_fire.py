import cv2
from ultralytics import YOLO

# 1. Load the OpenVINO model folder (trained YOLO11)
model = YOLO("best_openvino_model/")

# Calibration parameters for distance estimation (Pinhole Camera Model)
# Set to expected flame height (e.g., 5.0 cm for a candle / lighter)
KNOWN_FLAME_HEIGHT_CM = 5.0  
# Typical laptop webcam focal length in pixels (~600 px at 640x480 resolution)
FOCAL_LENGTH_PX = 600.0      

# 2. Open the webcam (0 is default webcam)
cap = cv2.VideoCapture(0)

print("Starting webcam fire detection test! (Press 'q' in the window to quit)")

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame from webcam.")
        break

    h, w, _ = frame.shape

    # 3. Run YOLO inference on the current frame
    results = model(frame, verbose=False)

    for result in results:
        boxes = result.boxes
        for box in boxes:
            conf = float(box.conf[0])
            
            # Confidence threshold: 40%
            if conf > 0.40:
                xyxy = box.xyxy[0].cpu().numpy().astype(int)
                x1, y1, x2, y2 = xyxy

                # --- Size Calculations ---
                box_w = max(x2 - x1, 1)
                box_h = max(y2 - y1, 1)
                box_area = box_w * box_h
                coverage_pct = (box_area / (w * h)) * 100

                # --- Center Coordinates (for robot aiming / steering) ---
                cx = int((x1 + x2) / 2)
                cy = int((y1 + y2) / 2)

                # --- Distance Estimation ---
                est_dist_cm = (KNOWN_FLAME_HEIGHT_CM * FOCAL_LENGTH_PX) / box_h

                # Draw bounding box (red) and center point (green)
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
                cv2.circle(frame, (cx, cy), 4, (0, 255, 0), -1)

                # Draw labels
                label_size = f"Fire: {conf:.2f} | Size: {box_w}x{box_h}px ({coverage_pct:.1f}%)"
                label_dist = f"Est. Dist: ~{est_dist_cm:.1f}cm (cx:{cx}, cy:{cy})"

                cv2.putText(frame, label_size, (x1, max(y1 - 25, 15)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                cv2.putText(frame, label_dist, (x1, max(y1 - 8, 30)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

    # 4. Display live feed
    cv2.imshow("Fire Detection - PC Webcam Test", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
print("Webcam test ended.")