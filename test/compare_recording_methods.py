import cv2
import numpy as np
import time
from datetime import datetime

class RecordingTester:
    def __init__(self):
        self.target_fps = 24
        self.frame_time = 1.0 / self.target_fps
        
    def create_test_frame(self, frame_num, method_name, actual_fps=0):
        """สร้างเฟรมทดสอบ"""
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        
        # พื้นหลัง
        cv2.rectangle(frame, (0, 0), (640, 80), (30, 30, 30), -1)
        
        # ข้อมูลหลัก
        cv2.putText(frame, f"Method: {method_name}", (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        cv2.putText(frame, f"Frame: {frame_num:04d}", (10, 120), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        # FPS info
        if actual_fps > 0:
            cv2.putText(frame, f"Processing FPS: {actual_fps:.1f}", (10, 160), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        
        # เวลา
        current_time = time.time()
        time_str = f"Time: {current_time:.3f}"
        cv2.putText(frame, time_str, (10, 200), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)
        
        # จุดเคลื่อนที่
        angle = (frame_num * 5) % 360
        center_x = 320 + int(100 * np.cos(np.radians(angle)))
        center_y = 300 + int(80 * np.sin(np.radians(angle)))
        cv2.circle(frame, (center_x, center_y), 20, (0, 0, 255), -1)
        
        return frame
    
    def test_fixed_fps_recording(self):
        """ทดสอบการบันทึกแบบ FPS คงที่ (แบบเก่า)"""
        print("=== Testing Fixed FPS Recording ===")
        
        timestamp = datetime.now().strftime("%H%M%S")
        filename = f"fixed_fps_test_{timestamp}.avi"
        
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        writer = cv2.VideoWriter(filename, fourcc, float(self.target_fps), (640, 480))
        
        if not writer.isOpened():
            print("❌ Cannot create fixed FPS video writer")
            return
        
        print(f"📹 Recording: {filename}")
        print(f"🎯 Fixed FPS: {self.target_fps}")
        
        start_time = time.time()
        last_frame_time = start_time
        last_record_time = start_time
        frame_count = 0
        record_count = 0
        
        # จำลองการประมวลผลที่มีความเร็วไม่คงที่
        processing_delays = [0.02, 0.05, 0.03, 0.08, 0.04]  # วินาที
        delay_index = 0
        
        duration = 5  # วินาที
        
        while time.time() - start_time < duration:
            current_time = time.time()
            
            # จำลองการประมวลผล
            time.sleep(processing_delays[delay_index % len(processing_delays)])
            delay_index += 1
            
            # Frame rate control สำหรับการประมวลผล
            if current_time - last_frame_time >= self.frame_time:
                frame = self.create_test_frame(frame_count, "Fixed FPS", 1.0/(current_time - last_frame_time))
                frame_count += 1
                last_frame_time = current_time
                
                # บันทึกตาม timing ที่กำหนด (แบบเก่า)
                if current_time - last_record_time >= self.frame_time:
                    writer.write(frame)
                    record_count += 1
                    last_record_time = current_time
        
        total_time = time.time() - start_time
        processing_fps = frame_count / total_time
        recorded_fps = record_count / total_time
        
        writer.release()
        
        print(f"✅ Completed:")
        print(f"   Duration: {total_time:.2f}s")
        print(f"   Processed frames: {frame_count}")
        print(f"   Recorded frames: {record_count}")
        print(f"   Processing FPS: {processing_fps:.2f}")
        print(f"   Recorded FPS: {recorded_fps:.2f}")
        print(f"   Target FPS: {self.target_fps}")
        
        return filename
    
    def test_adaptive_fps_recording(self):
        """ทดสอบการบันทึกแบบ FPS ตามการประมวลผลจริง (แบบใหม่)"""
        print("\n=== Testing Adaptive FPS Recording ===")
        
        timestamp = datetime.now().strftime("%H%M%S")
        filename = f"adaptive_fps_test_{timestamp}.avi"
        
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        # เริ่มด้วย FPS ประมาณการ
        initial_fps = 15  # เริ่มต่ำไว้
        writer = cv2.VideoWriter(filename, fourcc, float(initial_fps), (640, 480))
        
        if not writer.isOpened():
            print("❌ Cannot create adaptive FPS video writer")
            return
        
        print(f"📹 Recording: {filename}")
        print(f"🔄 Adaptive FPS (starts at {initial_fps})")
        
        start_time = time.time()
        last_frame_time = start_time
        frame_count = 0
        record_count = 0
        
        # จำลองการประมวลผลที่มีความเร็วไม่คงที่
        processing_delays = [0.02, 0.05, 0.03, 0.08, 0.04]
        delay_index = 0
        
        duration = 5  # วินาที
        
        while time.time() - start_time < duration:
            current_time = time.time()
            
            # จำลองการประมวลผล
            time.sleep(processing_delays[delay_index % len(processing_delays)])
            delay_index += 1
            
            # Frame rate control สำหรับการประมวลผล
            if current_time - last_frame_time >= self.frame_time:
                actual_frame_fps = 1.0 / (current_time - last_frame_time)
                frame = self.create_test_frame(frame_count, "Adaptive FPS", actual_frame_fps)
                frame_count += 1
                last_frame_time = current_time
                
                # บันทึกทุกเฟรมที่ประมวลผล (แบบใหม่)
                writer.write(frame)
                record_count += 1
        
        total_time = time.time() - start_time
        processing_fps = frame_count / total_time
        recorded_fps = record_count / total_time
        
        writer.release()
        
        print(f"✅ Completed:")
        print(f"   Duration: {total_time:.2f}s")
        print(f"   Processed frames: {frame_count}")
        print(f"   Recorded frames: {record_count}")
        print(f"   Processing FPS: {processing_fps:.2f}")
        print(f"   Recorded FPS: {recorded_fps:.2f}")
        print(f"   Frames match: {'YES' if frame_count == record_count else 'NO'}")
        
        return filename
    
    def analyze_video_files(self, fixed_file, adaptive_file):
        """วิเคราะห์ไฟล์วิดีโอที่สร้าง"""
        print(f"\n=== Video File Analysis ===")
        
        files = [
            (fixed_file, "Fixed FPS Method"),
            (adaptive_file, "Adaptive FPS Method")
        ]
        
        for filename, method in files:
            print(f"\n📁 {method}: {filename}")
            
            try:
                cap = cv2.VideoCapture(filename)
                if cap.isOpened():
                    fps = cap.get(cv2.CAP_PROP_FPS)
                    frame_count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
                    duration = frame_count / fps if fps > 0 else 0
                    
                    print(f"   📊 File FPS: {fps}")
                    print(f"   📊 Frame Count: {frame_count}")
                    print(f"   📊 Duration: {duration:.2f}s")
                    
                    cap.release()
                else:
                    print(f"   ❌ Cannot open file")
            except Exception as e:
                print(f"   ❌ Error: {e}")
    
    def run_comparison(self):
        """รันการเปรียบเทียบทั้งสองวิธี"""
        print("🎬 Recording Methods Comparison Test")
        print("=" * 50)
        
        # ทดสอบแบบเก่า (Fixed FPS)
        fixed_file = self.test_fixed_fps_recording()
        
        # หยุดสักครู่
        time.sleep(1)
        
        # ทดสอบแบบใหม่ (Adaptive FPS)
        adaptive_file = self.test_adaptive_fps_recording()
        
        # วิเคราะห์ผลลัพธ์
        if fixed_file and adaptive_file:
            self.analyze_video_files(fixed_file, adaptive_file)
        
        print(f"\n🎯 Summary:")
        print(f"   Fixed FPS: Records at predetermined frame rate (may skip/duplicate frames)")
        print(f"   Adaptive FPS: Records every processed frame (natural timing)")
        print(f"   Files saved in 'test/' folder for comparison")

if __name__ == "__main__":
    tester = RecordingTester()
    tester.run_comparison()
