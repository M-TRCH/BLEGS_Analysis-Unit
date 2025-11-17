import cv2
import numpy as np

# --- ตั้งค่า Marker ---
# เลือก Dictionary (ตระกูลของ Tag)
# DICT_6X6_250 คือตระกูลที่มี 250 Tag ขนาด 6x6 blocks
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)

# เลือก ID ของ Marker (เลือกได้ 0-249)
marker_id = 1 

# ขนาดของภาพ Marker ที่จะสร้าง (pixels)
img_size = 300 
#------------------------

# สร้าง Marker
marker_image = np.zeros((img_size, img_size), dtype=np.uint8)
marker_image = cv2.aruco.generateImageMarker(aruco_dict, marker_id, img_size, marker_image, 1)

# บันทึกเป็นไฟล์
cv2.imwrite("my_marker.png", marker_image)

print(f"สร้าง Marker ID={marker_id} ขนาด {img_size}x{img_size} pixels")
print("บันทึกเป็นไฟล์ 'my_marker.png' เรียบร้อย")

# แสดงผล (กด q เพื่อปิด)
cv2.imshow("ArUco Marker", marker_image)
cv2.waitKey(0)
cv2.destroyAllWindows()