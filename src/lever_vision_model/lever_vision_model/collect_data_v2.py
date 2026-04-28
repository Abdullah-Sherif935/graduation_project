import cv2
import os
import time

print("=== 🚀 Starting AUTO Data Collection Script ===")

# إنشاء المجلد
folder_name = "lever_v2_images"
os.makedirs(folder_name, exist_ok=True)
print(f"📁 Saving to folder: '{folder_name}'")

# فتح كاميرا الـ Orbbec (رقم 2)
cap = cv2.VideoCapture(2)

if not cap.isOpened():
    print("❌ Error: Could not open camera 2. Check connections.")
    exit()

count = 0
max_images = 50
capture_interval = 2.0  # الوقت بالثواني بين كل صورة والتانية (تقدر تغيره)

print("=== 📸 AUTO Mode ON ===")
print(f"- Capturing 1 image every {capture_interval} seconds.")
print("- Move the lever continuously.")
print("- Press 'q' to stop early.")
print("=======================")

# تسجيل وقت البداية
last_capture_time = time.time()

while count < max_images:
    success, frame = cap.read()
    if not success:
        print("Failed to grab frame.")
        break

    display_frame = frame.copy()
    current_time = time.time()
    
    # حساب الوقت اللي عدى
    time_elapsed = current_time - last_capture_time
    time_left = max(0, capture_interval - time_elapsed)

    # رسم واجهة المستخدم على الشاشة
    cv2.putText(display_frame, f"Captured: {count}/{max_images}", (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    cv2.putText(display_frame, f"Next pic in: {time_left:.1f}s", (20, 80),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
    cv2.putText(display_frame, "Press 'q' to Quit", (20, 120),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

    cv2.imshow("Auto Data Collection - V2", display_frame)

    # التقاط الصورة أوتوماتيكياً لما الثواني تخلص
    if time_elapsed >= capture_interval:
        img_path = os.path.join(folder_name, f"dynamic_lever_{count+1}.jpg")
        cv2.imwrite(img_path, frame)
        print(f"✅ Image {count+1} saved!")
        count += 1
        last_capture_time = current_time  # تصفير العداد للصورة اللي بعدها

    # الخروج لو ضغطت 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        print("⚠️ Stopped by user.")
        break

print(f"\n🎉 Done! {count} images saved automatically inside '{folder_name}'.")

cap.release()
cv2.destroyAllWindows()
