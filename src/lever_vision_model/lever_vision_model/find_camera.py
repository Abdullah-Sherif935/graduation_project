import cv2
print("🔍 Scanning for available cameras...")
for i in range(10):
    cap = cv2.VideoCapture(i)
    if cap.isOpened():
        print(f"✅ Camera found and working at index: {i}")
        cap.release()
print("Scan complete.")
