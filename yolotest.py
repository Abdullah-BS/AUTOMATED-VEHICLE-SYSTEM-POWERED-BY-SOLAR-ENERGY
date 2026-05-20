#!/usr/bin/env python3

import cv2
from ultralytics import YOLO

CAMERA_INDEX = 0
MODEL_NAME = "yolo11n.pt"
CONFIDENCE = 0.50
PERSON_CLASS = 0
WINDOW_TITLE = "YOLO Human Detection | Press Q to quit"

BOX_COLOR = (0, 255, 100)
LABEL_BG_COLOR = (0, 200, 80)
TEXT_COLOR = (0, 0, 0)
FONT = cv2.FONT_HERSHEY_SIMPLEX
FONT_SCALE = 0.6
FONT_THICKNESS = 2
LINE_THICKNESS = 2

print(f"[INFO] Loading model: {MODEL_NAME}")
model = YOLO(MODEL_NAME)
print("[INFO] Model loaded successfully.")

cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_V4L2)

if not cap.isOpened():
    print(f"[ERROR] Cannot open camera at index {CAMERA_INDEX}.")
    exit(1)

cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cap.set(cv2.CAP_PROP_FPS, 30)

actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
print(f"[INFO] Camera opened — resolution: {actual_w}x{actual_h}")

for i in range(20):
    ret, frame = cap.read()
    if ret and frame is not None:
        print(f"[INFO] Camera warm-up success on try {i+1}")
        break
else:
    print("[ERROR] Camera opened but no frames were received.")
    cap.release()
    exit(1)

print("[INFO] Press Q or ESC to quit.\n")

while True:
    ret, frame = cap.read()
    if not ret or frame is None:
        print("[WARNING] Failed to read frame from camera.")
        continue

    results = model(
        frame,
        classes=[PERSON_CLASS],
        conf=CONFIDENCE,
        verbose=False
    )

    person_count = 0

    for result in results:
        for box in result.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            conf_score = float(box.conf[0])
            person_count += 1

            cv2.rectangle(frame, (x1, y1), (x2, y2), BOX_COLOR, LINE_THICKNESS)

            label = f"Person {conf_score:.0%}"
            (tw, th), baseline = cv2.getTextSize(label, FONT, FONT_SCALE, FONT_THICKNESS)
            label_y = max(y1 - 6, th + 4)

            cv2.rectangle(
                frame,
                (x1, label_y - th - 4),
                (x1 + tw + 6, label_y + baseline),
                LABEL_BG_COLOR,
                cv2.FILLED
            )

            cv2.putText(
                frame,
                label,
                (x1 + 3, label_y),
                FONT,
                FONT_SCALE,
                TEXT_COLOR,
                FONT_THICKNESS,
                cv2.LINE_AA
            )

    hud = f"Detected: {person_count} person{'s' if person_count != 1 else ''} | Conf >= {CONFIDENCE:.0%}"
    cv2.putText(frame, hud, (10, 30), FONT, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
    cv2.putText(frame, hud, (10, 30), FONT, 0.7, (30, 30, 30), 1, cv2.LINE_AA)

    cv2.imshow(WINDOW_TITLE, frame)
    key = cv2.waitKey(1) & 0xFF

    if key in (ord('q'), ord('Q'), 27):
        print("[INFO] Quit signal received.")
        break

cap.release()
cv2.destroyAllWindows()
print("[INFO] Camera released. Goodbye!")