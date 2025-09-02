import cv2

cap = cv2.VideoCapture('test/adaptive_blob_detection_20250902_164349.avi')
fps = cap.get(cv2.CAP_PROP_FPS)
frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
duration = frames / fps if fps > 0 else 0

print(f"Adaptive Blob Detection Video:")
print(f"FPS: {fps}")
print(f"Frames: {frames}")
print(f"Duration: {duration:.2f}s")

cap.release()
