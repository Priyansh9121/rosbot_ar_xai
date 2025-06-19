import cv2
import numpy as np

cap = cv2.VideoCapture(0)  # Replace with a sample video path if needed
while True:
    ret, frame = cap.read()
    if not ret:
        print("No frame")
        break
    cv2.imshow("Test", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
cap.release()
cv2.destroyAllWindows()
