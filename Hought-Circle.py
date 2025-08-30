import cv2
import numpy as np
import time

class GreenCircleDetector:
    def __init__(self):
        # กำหนดช่วงกว้างของสีเขียวนีออนใน HSV
        # สีเป้าหมาย RGB = (165, 224, 155) แปลงเป็น HSV ≈ (98, 81, 88) (OpenCV scale: H=49, S=80, V=224)
        # กำหนดช่วงเผื่อ H, S, V เพื่อให้ตรวจจับได้แม่นยำและเผื่อความคลาดเคลื่อนของแสง
        h, s, v = 56, 168, 176  # ค่าประมาณจาก (165, 224, 155)
        h_margin = 10
        s_margin = 40
        v_margin = 30
        self.lower_green = np.array([h - h_margin, max(0, s - s_margin), max(0, v - v_margin)])
        self.upper_green = np.array([h + h_margin, min(255, s + s_margin), min(255, v + v_margin)])
        
        # พารามิเตอร์สำหรับ HoughCircles
        self.dp = 1.2
        self.min_dist = 50      # ระยะห่างขั้นต่ำระหว่างวงกลม
        self.param1 = 50        # พารามิเตอร์แรกสำหรับ Canny edge detection
        self.param2 = 30        # พารามิเตอร์ที่สองสำหรับ HoughCircles
        self.min_radius = 10    
        self.max_radius = 80
        
        # สำหรับคำนวณ FPS
        self.fps_counter = 0
        self.fps_start_time = time.time()
        self.current_fps = 0
    
    def preprocess_frame(self, frame):
        """ปรับปรุงภาพก่อนการตรวจจับ"""
        # แปลงเป็น HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # สร้าง mask สำหรับสีเขียว
        mask = cv2.inRange(hsv, self.lower_green, self.upper_green)
        
        # ใช้ Gaussian Blur ลด noise
        mask = cv2.GaussianBlur(mask, (5, 5), 0)
        
        # Morphological operations เพื่อปิดช่องว่างและกำจัด noise
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        return mask
    
    def detect_circles(self, mask):
        """ตรวจจับวงกลมใน mask"""
        circles = cv2.HoughCircles(
            mask, 
            cv2.HOUGH_GRADIENT,
            dp=self.dp,
            minDist=self.min_dist,
            param1=self.param1,
            param2=self.param2,
            minRadius=self.min_radius,
            maxRadius=self.max_radius
        )
        
        return circles
    
    def draw_results(self, frame, circles, mask):
        """วาดผลลัพธ์บนเฟรม"""
        result_frame = frame.copy()
        
        if circles is not None:
            circles = np.round(circles[0, :]).astype("int")
            
            for (x, y, r) in circles:
                # วาดวงกลมที่ตรวจจับได้
                cv2.circle(result_frame, (x, y), r, (0, 255, 0), 3)
                cv2.circle(result_frame, (x, y), 2, (0, 0, 255), 3)
                
                # แสดงข้อมูลตำแหน่งและรัศมี
                cv2.putText(result_frame, f"R:{r} X:{x} Y:{y}", 
                           (x-50, y-r-10), cv2.FONT_HERSHEY_SIMPLEX, 
                           0.5, (255, 255, 255), 2)
        
        # แสดง FPS
        cv2.putText(result_frame, f"FPS: {self.current_fps:.1f}", 
                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
        
        return result_frame
    
    def calculate_fps(self):
        """คำนวณ FPS"""
        self.fps_counter += 1
        if self.fps_counter >= 30:
            end_time = time.time()
            self.current_fps = 30 / (end_time - self.fps_start_time)
            self.fps_counter = 0
            self.fps_start_time = time.time()

def main():
    # เปิดกล้อง
    cap = cv2.VideoCapture(0)
    
    # ตั้งค่าความละเอียด
    width, height = 1280, 720
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, 30)  # ตั้งค่า FPS
    
    # สร้าง detector
    detector = GreenCircleDetector()
    
    print("กำลังเปิดกล้อง... กด 'q' เพื่อออก, 's' เพื่อบันทึกภาพ")
    print("กด 'h' เพื่อแสดง/ซ่อน mask, 'c' เพื่อปรับตั้งค่า")
    
    show_mask = False
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("ไม่สามารถอ่านข้อมูลจากกล้องได้")
            break
        
        # ปรับขนาดเฟรม
        frame = cv2.resize(frame, (width, height))
        
        # ประมวลผลภาพ
        mask = detector.preprocess_frame(frame)
        circles = detector.detect_circles(mask)
        result_frame = detector.draw_results(frame, circles, mask)
        
        # คำนวณ FPS
        detector.calculate_fps()
        
        # แสดงผล
        cv2.imshow('Green Circle Detection', result_frame)
        
        if show_mask:
            cv2.imshow('Green Mask', mask)
        
        # จัดการ key input
        key = cv2.waitKey(1) & 0xFF
        
        if key == ord('q'):  # ออกจากโปรแกรม
            break
        elif key == ord('s'):  # บันทึกภาพ
            timestamp = int(time.time())
            cv2.imwrite(f'detected_circles_{timestamp}.jpg', result_frame)
            print(f"บันทึกภาพแล้ว: detected_circles_{timestamp}.jpg")
        elif key == ord('h'):  # แสดง/ซ่อน mask
            show_mask = not show_mask
            if not show_mask:
                cv2.destroyWindow('Green Mask')
        elif key == ord('c'):  # แสดงค่าปัจจุบัน
            print("\n--- การตั้งค่าปัจจุบัน ---")
            print(f"Green HSV Range: {detector.lower_green} - {detector.upper_green}")
            print(f"Circle Radius: {detector.min_radius} - {detector.max_radius}")
            print(f"Min Distance: {detector.min_dist}")
    
    # ปิดการใช้งาน
    cap.release()
    cv2.destroyAllWindows()
    print("ปิดกล้องแล้ว")

if __name__ == "__main__":
    main()