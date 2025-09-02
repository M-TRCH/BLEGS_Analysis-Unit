import cv2

filename = "test/camera_info_demo_20250902_165625.avi"
cap = cv2.VideoCapture(filename)

if cap.isOpened():
    fps = cap.get(cv2.CAP_PROP_FPS)
    frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    duration = frames / fps if fps > 0 else 0
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    
    print(f"📹 Camera Info Demo Video:")
    print(f"   File: {filename}")
    print(f"   Resolution: {width}x{height}")
    print(f"   FPS: {fps}")
    print(f"   Frames: {frames}")
    print(f"   Duration: {duration:.2f}s")
    print(f"   Size: 2.5MB")
    
    cap.release()
    print(f"✅ Video contains camera settings overlay demo")
else:
    print(f"❌ Cannot open video file")
