import cv2
import numpy as np

# --- 1. ตั้งค่าพื้นฐาน ---
# เลือก Dictionary ให้ตรงกับ Tag ที่คุณสร้าง (DICT_6X6_250)
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
aruco_params = cv2.aruco.DetectorParameters()

# สร้าง Detector (สำหรับ OpenCV 4.7+)
detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

# --- 2. ตั้งค่าขนาด Marker (สำคัญมาก!) ---
# วัดขนาดจริงของ Marker ที่คุณ "พิมพ์" ออกมา (วัดเฉพาะส่วนสีดำ)
# หน่วยเป็น "เมตร"
MARKER_SIZE_METERS = 0.05  # ตัวอย่าง: 5 เซนติเมตร (คุณต้องเปลี่ยนค่านี้)

# --- 3. Dummy Camera Calibration (สำหรับการทดสอบ) ---
# 
# !! คำเตือน !!
# ค่านี้เป็นค่า "ปลอม" สำหรับทดสอบเท่านั้น
# หากต้องการความแม่นยำ 3D จริง คุณต้องทำการ Calibrate กล้องของคุณก่อน
# 
# กล้องมีความละเอียด 3840x2160 (4K)
focal_length = 2000  # ประมาณการจาก resolution (ปกติ focal ~ resolution)
center_x = 3840 / 2
center_y = 2160 / 2

# Camera Matrix (K)
camera_matrix = np.array([
    [focal_length, 0, center_x],
    [0, focal_length, center_y],
    [0, 0, 1]
], dtype=np.float32)

# Distortion Coefficients (D) - สมมติว่าไม่มีการบิดเบือน
dist_coeffs = np.zeros((5, 1), dtype=np.float32)

print("ใช้ Camera Matrix (Dummy):")
print(camera_matrix)
print("\nใช้ Distortion Coeffs (Dummy):")
print(dist_coeffs)
#----------------------------------------------------

# --- 4. เปิดกล้อง ---
cap = cv2.VideoCapture(2) # 0 คือกล้องตัวแรก
if not cap.isOpened():
    print("ไม่สามารถเปิดกล้องได้")
    exit()

# ตั้งค่าความละเอียดกล้องเป็น 3840x2160 (4K)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 3840)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 2160)

# อ่านความละเอียดจริงที่ได้
actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
print(f"\nความละเอียดกล้อง: {actual_width}x{actual_height}")

print("\nกำลังเปิดกล้อง... กด 'q' เพื่อออกจากโปรแกรม")

# --- 5. ลูปหลัก ---
while True:
    ret, frame = cap.read()
    if not ret:
        break
    
    # แปลงเป็น Grayscale (ดีต่อการประมวลผล)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # ตรวจจับ Markers (ใช้ detector สำหรับ OpenCV 4.7+)
    corners, ids, rejected = detector.detectMarkers(gray)
    
    # ถ้าเจออย่างน้อย 1 Marker
    if ids is not None:
        # วาดกรอบสี่เหลี่ยมรอบ Marker ที่เจอ
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        
        # คำนวณตำแหน่ง (Pose) ของแต่ละ Marker
        # ใช้ API ใหม่ของ OpenCV 4.7+
        for i in range(len(ids)):
            # สร้าง object points (3D coordinates ของ marker corners)
            objPoints = np.array([
                [-MARKER_SIZE_METERS/2, MARKER_SIZE_METERS/2, 0],
                [MARKER_SIZE_METERS/2, MARKER_SIZE_METERS/2, 0],
                [MARKER_SIZE_METERS/2, -MARKER_SIZE_METERS/2, 0],
                [-MARKER_SIZE_METERS/2, -MARKER_SIZE_METERS/2, 0]
            ], dtype=np.float32)
            
            # คำนวณ pose
            success, rvec, tvec = cv2.solvePnP(
                objPoints,
                corners[i][0],
                camera_matrix,
                dist_coeffs,
                flags=cv2.SOLVEPNP_IPPE_SQUARE
            )
            
            if success:
                # วาดแกน (แกนสี แดง=X, เขียว=Y, น้ำเงิน=Z)
                cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, 0.025)
                
                # แสดงข้อมูล
                tvec_flat = tvec.flatten()
                marker_id = ids[i][0]
                
                # แสดงข้อมูลบนหน้าจอ
                text = f"ID:{marker_id} Z:{tvec_flat[2]:.2f}m"
                # หาตำแหน่งกึ่งกลางของ marker
                center = corners[i][0].mean(axis=0).astype(int)
                cv2.putText(frame, text, tuple(center), cv2.FONT_HERSHEY_SIMPLEX, 
                           0.5, (0, 255, 0), 2)
                
                print(f"ID: {marker_id} | X: {tvec_flat[0]:.3f} Y: {tvec_flat[1]:.3f} Z: {tvec_flat[2]:.3f} (เมตร)")

    # แสดงผลลัพธ์
    cv2.imshow("ArUco Detection", frame)
    
    # กด 'q' เพื่อออก
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# --- 6. คืนค่ากล้องและปิดหน้าต่าง ---
cap.release()
cv2.destroyAllWindows()
print("ปิดโปรแกรม")