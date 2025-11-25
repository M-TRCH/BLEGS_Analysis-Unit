import cv2
import numpy as np
import os

# --- ตั้งค่า ---
# จำนวนจุดตัดด้านใน (แนวนอน, แนวตั้ง) *ไม่ใช่จำนวนช่องสี่เหลี่ยม*
CHECKERBOARD_DIMS = (9, 6) 

# Index กล้อง (เปลี่ยนเลขถ้าเปิดไม่ติด)
CAMERA_INDEX = 1  

# โฟลเดอร์ที่จะเก็บภาพ
IMAGE_DIR = "calib_images"

# ความละเอียดกล้อง (ตั้งให้ตรงกับที่ใช้จริง)
FRAME_W = 3840
FRAME_H = 2160
# -------------

# สร้างโฟลเดอร์เก็บภาพ
if not os.path.exists(IMAGE_DIR):
    os.makedirs(IMAGE_DIR)

# เปิดกล้อง
cap = cv2.VideoCapture(CAMERA_INDEX)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_W)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)

# เช็คความละเอียดจริง
act_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
act_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
print(f"กล้องเปิดที่ความละเอียด: {act_w}x{act_h}")
print("------------------------------------------")
print("กด [SPACE] เพื่อบันทึกภาพ")
print("กด [q] เพื่อจบการทำงาน")
print("------------------------------------------")

count = 0

while True:
    ret, frame = cap.read()
    if not ret:
        print("ไม่พบสัญญาณภาพ")
        break

    # แปลงเป็นขาวดำเพื่อหาจุดตัด (ใช้แสดงผลเฉยๆ)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    ret_corn, corners = cv2.findChessboardCorners(gray, CHECKERBOARD_DIMS, None)

    # ทำสำเนาเพื่อวาดเส้น (ไม่วาดทับภาพจริง)
    preview_frame = frame.copy()

    if ret_corn:
        # วาดจุดตัดที่เจอ
        cv2.drawChessboardCorners(preview_frame, CHECKERBOARD_DIMS, corners, ret_corn)
        # ขึ้นข้อความว่า "READY"
        cv2.putText(preview_frame, "READY TO CAPTURE", (50, 100), 
                    cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 3)

    # ย่อภาพแสดงผล (เพราะจอคอมรับ 4K ไม่ไหว)
    display_img = cv2.resize(preview_frame, (1280, 720))
    cv2.imshow("Camera Calibration Capture", display_img)

    key = cv2.waitKey(1)

    if key == ord(' '): # กด Spacebar
        if ret_corn:
            img_name = os.path.join(IMAGE_DIR, f"calib_{count:03d}.png")
            # บันทึกภาพต้นฉบับ (Original 4K)
            cv2.imwrite(img_name, frame)
            print(f"บันทึกภาพ: {img_name}")
            count += 1
        else:
            print("หาตารางไม่เจอ! กรุณาปรับมุม")
            
    elif key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
print(f"เสร็จสิ้น... บันทึกไปทั้งหมด {count} ภาพ")