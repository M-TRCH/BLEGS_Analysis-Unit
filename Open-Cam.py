import cv2

cap = cv2.VideoCapture(0)  # ใช้กล้องตัวแรก (0)

width, height = 1280, 720

cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # ปรับขนาดเฟรมให้เป็น 16:9 และย่อครึ่ง
    frame = cv2.resize(frame, (width, height))

    cv2.imshow('Camera', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):  # กด q
        break

cap.release()
cv2.destroyAllWindows()