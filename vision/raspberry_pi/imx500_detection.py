from pathlib import Path

import cv2
from picamera2 import Picamera2
from picamera2.devices.imx500 import IMX500

# 1. Initialize the IMX500 AI Camera helper with your compiled model
MODEL_PATH = str(Path(__file__).resolve().parents[2] / "models/imx500/fire_network.rpk")
imx500 = IMX500(MODEL_PATH)

# Calibration parameters for distance estimation (Pinhole Camera Model)
# Set KNOWN_FLAME_HEIGHT_CM to the expected height of your target flame (e.g. candle ~3-5 cm, burner ~10 cm)
KNOWN_FLAME_HEIGHT_CM = 5.0  
# Estimated focal length in pixels for IMX500 at 640x640 (HFOV ~66 deg -> ~490 px)
FOCAL_LENGTH_PX = 490.0

# 2. Initialize the camera
picam2 = Picamera2()

# 3. Configure the camera resolution (640x640 matches YOLO training)
config = picam2.create_preview_configuration(main={"size": (640, 640)})
picam2.configure(config)
picam2.start()

print("AI Camera started! Looking for fire... (Press 'q' on the video window to quit)")

try:
    # 4. Main Inference Loop
    while True:
        # Capture the latest image frame and the AI metadata from the NPU
        request = picam2.capture_request()
        image = request.make_array("main")
        metadata = request.metadata
        
        # 5. Extract bounding boxes from the IMX500 metadata
        outputs = imx500.get_outputs(metadata)
        
        if outputs and len(outputs) > 0:
            for detection in outputs[0]: 
                box = detection.get('box', [0, 0, 0, 0])
                score = detection.get('score', 0.0)
                
                # Confidence threshold (e.g., 40%)
                if score > 0.40:  
                    h, w, _ = image.shape
                    
                    # Convert normalized coordinates (0.0 - 1.0) to pixel coordinates
                    x1 = int(box[0] * w)
                    y1 = int(box[1] * h)
                    x2 = int(box[2] * w)
                    y2 = int(box[3] * h)
                    
                    # --- Size Calculations ---
                    box_w = max(x2 - x1, 1)
                    box_h = max(y2 - y1, 1)
                    box_area = box_w * box_h
                    coverage_pct = (box_area / (w * h)) * 100

                    # --- Center Coordinates (for robot aiming / steering) ---
                    cx = int((x1 + x2) / 2)
                    cy = int((y1 + y2) / 2)

                    # --- Distance Estimation (Pinhole formula) ---
                    est_distance_cm = (KNOWN_FLAME_HEIGHT_CM * FOCAL_LENGTH_PX) / box_h

                    # Draw bounding box (red) and center point (green)
                    cv2.rectangle(image, (x1, y1), (x2, y2), (0, 0, 255), 2)
                    cv2.circle(image, (cx, cy), 4, (0, 255, 0), -1)
                    
                    # Overlay size and distance labels
                    label_score_size = f"Fire: {score:.2f} | Size: {box_w}x{box_h}px ({coverage_pct:.1f}%)"
                    label_dist = f"Est. Dist: ~{est_distance_cm:.1f}cm (cx:{cx}, cy:{cy})"
                    
                    cv2.putText(image, label_score_size, (x1, max(y1 - 25, 15)), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                    cv2.putText(image, label_dist, (x1, max(y1 - 8, 30)), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

        # 6. Display the live feed
        cv2.imshow("Raspberry Pi AI Camera", image)
        
        # Press 'q' to exit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    picam2.stop()
    cv2.destroyAllWindows()
    print("Camera safely shut down.")