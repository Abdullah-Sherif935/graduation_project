from ultralytics import YOLO
from roboflow import Roboflow

# 1. تحميل الداتا سيت الجديدة (V2)
rf = Roboflow(api_key="u1ZjkmHJxqTyo8v81QdU")
project = rf.workspace("abdullahs-workspace-sseko").project("elevator_lever_pose")
version = project.version(2)
dataset = version.download("yolov8")

# 2. تحميل موديل YOLOv8-Pose الأساسي
model = YOLO('yolov8n-pose.pt')

# 3. بدء التدريب المطور
results = model.train(
    data=f"{dataset.location}/data.yaml",
    epochs=50,
    imgsz=640,
    device='cpu',
    # --- تعديل الـ Augmentation للأسماء الصحيحة ---
    degrees=15.0,    # دوران 15 درجة يمين وشمال
    fliplr=0.5,      # عكس الصورة يمين وشمال (عشان الأبواب اليمين)
    hsv_v=0.4,       # ده بديل الـ brightness (بيغير في قيمة الإضاءة)
    hsv_s=0.7,       # بيغير في تشبع الألوان (عشان لو الإضاءة صفراء أو بيضاء)
    # ------------------------------------------
    name='lever_v2_model' #اسم الفولدر الجديد
)
print("=== Training Complete! Your 'v2' model is ready ===")
