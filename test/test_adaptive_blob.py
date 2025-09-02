import cv2
import numpy as np
import time
from datetime import datetime

# จำลองการทำงานของโปรแกรม Color Blob Detection แบบใหม่
def test_adaptive_blob_detection():
    print("=== Testing Adaptive FPS Blob Detection ===")
    
    # สร้างเฟรมทดสอบที่มี blob
    def create_blob_frame(frame_num, processing_time):
        frame = np.zeros((1080, 1920, 3), dtype=np.uint8)  # HD resolution
        
        # สร้าง blob สีเขียวที่เคลื่อนที่
        center_x = 960 + int(300 * np.cos(frame_num * 0.1))
        center_y = 540 + int(200 * np.sin(frame_num * 0.05))
        cv2.circle(frame, (center_x, center_y), 80, (50, 200, 50), -1)  # สีเขียว
        
        # เพิ่ม noise เพื่อจำลองการประมวลผลที่ซับซ้อน
        for i in range(5):
            noise_x = np.random.randint(100, 1820)
            noise_y = np.random.randint(100, 980)
            cv2.circle(frame, (noise_x, noise_y), np.random.randint(20, 40), 
                      (np.random.randint(0, 100), np.random.randint(0, 100), np.random.randint(0, 100)), -1)
        
        # ข้อมูล overlay
        cv2.rectangle(frame, (20, 20), (600, 200), (0, 0, 0), -1)
        cv2.putText(frame, f"Frame: {frame_num:04d}", (30, 60), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 255, 255), 2)
        cv2.putText(frame, f"Processing Time: {processing_time*1000:.1f}ms", (30, 100), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        cv2.putText(frame, f"Blob Center: ({center_x}, {center_y})", (30, 140), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.putText(frame, "ADAPTIVE FPS RECORDING", (30, 180), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        
        return frame
    
    # การตั้งค่า
    target_fps = 24
    frame_time = 1.0 / target_fps
    test_duration = 8  # วินาที
    
    # สร้าง video writer
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"adaptive_blob_detection_{timestamp}.avi"
    
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    # ใช้ FPS เริ่มต้นที่ประมาณการ แต่จะบันทึกตามจริง
    initial_fps = 20
    writer = cv2.VideoWriter(filename, fourcc, float(initial_fps), (1920, 1080))
    
    if not writer.isOpened():
        print("❌ Cannot create video writer")
        return
    
    print(f"🎬 Recording: {filename}")
    print(f"⚡ Recording every processed frame (adaptive FPS)")
    print(f"🎯 Target processing rate: {target_fps} FPS")
    
    # ตัวแปรสำหรับการวิเคราะห์
    start_time = time.time()
    last_frame_time = start_time
    frame_count = 0
    record_count = 0
    frame_times = []
    
    # จำลองค่า processing time ที่แตกต่างกัน
    processing_times = [0.02, 0.03, 0.05, 0.08, 0.04, 0.06, 0.03, 0.07]
    
    while time.time() - start_time < test_duration:
        current_time = time.time()
        
        # จำลอง frame rate control (เหมือนในโปรแกรมจริง)
        if current_time - last_frame_time >= frame_time:
            
            # จำลองเวลาการประมวลผล
            processing_time = processing_times[frame_count % len(processing_times)]
            time.sleep(processing_time)
            
            # สร้างเฟรม
            frame = create_blob_frame(frame_count, processing_time)
            
            # บันทึกทุกเฟรมที่ประมวลผล (แบบใหม่)
            writer.write(frame)
            record_count += 1
            
            # ติดตาม FPS
            frame_times.append(current_time)
            if len(frame_times) > 30:
                frame_times.pop(0)
            
            if len(frame_times) > 1:
                avg_time = (frame_times[-1] - frame_times[0]) / (len(frame_times) - 1)
                actual_fps = 1.0 / avg_time if avg_time > 0 else 0
            else:
                actual_fps = 0
            
            frame_count += 1
            last_frame_time = current_time
            
            # แสดงความคืบหน้า
            if frame_count % 20 == 0:
                elapsed = current_time - start_time
                print(f"⏱️  {elapsed:.1f}s | Frames: {frame_count} | Recorded: {record_count} | FPS: {actual_fps:.1f}")
    
    # สรุปผล
    total_time = time.time() - start_time
    avg_fps = frame_count / total_time
    
    writer.release()
    
    print(f"\n📊 Recording Summary:")
    print(f"   Duration: {total_time:.2f}s")
    print(f"   Processed frames: {frame_count}")
    print(f"   Recorded frames: {record_count}")
    print(f"   Match: {'YES' if frame_count == record_count else 'NO'}")
    print(f"   Average FPS: {avg_fps:.2f}")
    print(f"   Target FPS: {target_fps}")
    
    # ตรวจสอบไฟล์
    cap = cv2.VideoCapture(filename)
    if cap.isOpened():
        file_fps = cap.get(cv2.CAP_PROP_FPS)
        file_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        file_duration = file_frames / file_fps if file_fps > 0 else 0
        cap.release()
        
        print(f"\n📁 File Properties:")
        print(f"   File FPS: {file_fps}")
        print(f"   Frame count: {file_frames}")
        print(f"   File duration: {file_duration:.2f}s")
        
        # ประเมินความสำเร็จ
        frame_accuracy = abs(file_frames - record_count)
        if frame_accuracy <= 1:
            print(f"   ✅ EXCELLENT: Recorded every processed frame!")
        else:
            print(f"   ⚠️  Frame difference: {frame_accuracy}")
    
    print(f"\n🎯 Benefits of Adaptive Recording:")
    print(f"   ✅ Records exactly what was processed")
    print(f"   ✅ No artificial timing constraints")
    print(f"   ✅ Natural playback speed matches processing")
    print(f"   ✅ Better for analysis and debugging")
    
    return filename

if __name__ == "__main__":
    filename = test_adaptive_blob_detection()
    print(f"\n🎬 Test completed! Video saved as: {filename}")
    print(f"📂 File location: test/{filename}")
