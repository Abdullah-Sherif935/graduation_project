import cv2
import math
from ultralytics import YOLO

print("=== Loading Your V2 Lever Model ===")
# تحميل أوزان الموديل الجديد (V2)
model = YOLO('runs/pose/lever_v2_model/weights/best.pt')

print("=== Opening Orbbec Camera ===")
# فتح كاميرا الأوربيك (تأكد إن رقمها 2)
cap = cv2.VideoCapture(2)

if not cap.isOpened():
    print("❌ Error: Could not open camera.")
    exit()

print("=== Starting Real-time Tracking & Kinematics ===")

while cap.isOpened():
    success, frame = cap.read()
    if not success:
        print("Failed to grab frame.")
        break

    # 1. تشغيل الذكاء الاصطناعي على الفريم
    results = model(frame, verbose=False)
    
    # 2. رسم المربعات والنقط الأساسية
    annotated_frame = results[0].plot()

    # 3. استخراج الإحداثيات الحقيقية وحساب الزاوية للروبوت
    if results[0].keypoints is not None and len(results[0].keypoints.xy) > 0:
        keypoints = results[0].keypoints.xy[0].cpu().numpy()
        
        # التأكد إن الموديل لقط النقطتين بوضوح
        if len(keypoints) >= 2 and keypoints[0][0] != 0 and keypoints[1][0] != 0:
            p1_x, p1_y = int(keypoints[0][0]), int(keypoints[0][1]) # نقطة المحور (Pivot)
            p2_x, p2_y = int(keypoints[1][0]), int(keypoints[1][1]) # نقطة الطرف (Tip)

            # حساب المسافات بين المحور والطرف
            dx = p2_x - p1_x
            dy = p2_y - p1_y 

            # --- بداية الذكاء الرياضي لتعديل الزاوية (Smart Mapping) ---
            # حساب الزاوية الخام من الـ OpenCV
            raw_angle = math.degrees(math.atan2(dy, dx))

            # تعديل الزاوية للروبوت (عكسنا الإشارات هنا)
            if dx < 0: 
                # حالة الباب اللي أوكرته بتبص ناحية الشمال 
                if raw_angle > 0:
                    angle = -(180.0 - raw_angle)  # ضغطة لتحت (هتبقى بالسالب دلوقتي)
                else:
                    angle = (180.0 + raw_angle)   # رفعة لفوق (هتبقى بالموجب)
            else:
                # حالة الباب اللي أوكرته بتبص ناحية اليمين
                angle = -raw_angle  # بنعكسها برضه عشان تمشي مع نفس النظام
            
            # --- ملاحظة: لو عايز الرقم دايماً موجب (بدون سوالب خالص) شيل الـ # من السطر اللي تحت:
            # angle = abs(angle)
            # --- نهاية الذكاء الرياضي ---

            # طباعة الإحداثيات والزاوية المعدلة على الشاشة
            cv2.putText(annotated_frame, f"Pivot (X,Y): {p1_x}, {p1_y}", (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(annotated_frame, f"Target Tip (X,Y): {p2_x}, {p2_y}", (10, 60), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            cv2.putText(annotated_frame, f"Angle for ROS: {angle:.1f} deg", (10, 90), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

            # طباعة نظيفة في التيرمينال عشان نراقب الداتا اللي هتروح للـ ROS2
            print(f"🎯 Target: ({p2_x},{p2_y}) | Angle: {angle:.1f}°")

    # عرض النتيجة اللايف
    cv2.imshow("Lever Tracker - V2 (ROS Ready)", annotated_frame)

    # الخروج بالضغط على حرف q
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# التنظيف والإغلاق
cap.release()
cv2.destroyAllWindows()
