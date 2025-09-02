import cv2

print("=== Video Analysis Results ===")

files = [
    ("test/fixed_fps_test_164145.avi", "Fixed FPS Method (แบบเก่า)"),
    ("test/adaptive_fps_test_164151.avi", "Adaptive FPS Method (แบบใหม่)")
]

for filename, method in files:
    print(f"\n📁 {method}")
    print(f"   File: {filename}")
    
    try:
        cap = cv2.VideoCapture(filename)
        if cap.isOpened():
            fps = cap.get(cv2.CAP_PROP_FPS)
            frame_count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
            duration = frame_count / fps if fps > 0 else 0
            width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            
            print(f"   📊 Properties:")
            print(f"      File FPS: {fps}")
            print(f"      Frame Count: {frame_count}")
            print(f"      Duration: {duration:.2f}s")
            print(f"      Resolution: {width}x{height}")
            
            # คำนวณ actual frame rate
            actual_fps = frame_count / 5.0  # ทดสอบ 5 วินาที
            print(f"      Calculated FPS: {actual_fps:.2f}")
            
            cap.release()
        else:
            print(f"   ❌ Cannot open file")
    except Exception as e:
        print(f"   ❌ Error: {e}")

print(f"\n🎯 Key Differences:")
print(f"   📹 Fixed FPS: บันทึกตาม frame rate ที่กำหนดไว้ (24 FPS)")
print(f"   🔄 Adaptive FPS: บันทึกตามความเร็วการประมวลผลจริง")
print(f"   ⚡ Adaptive method สามารถจับความเปลี่ยนแปลงของการประมวลผลได้ดีกว่า")
print(f"   🎬 Video จะเล่นตามจังหวะการประมวลผลจริงที่เกิดขึ้น")
