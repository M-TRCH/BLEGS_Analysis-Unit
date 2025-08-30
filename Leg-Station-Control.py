import serial
import time


def send_position(x, y):
    cmd = f"# {x} {y}\n"
    ser.write(cmd.encode())
    print(f"Sent: {cmd.strip()}")

def run_line_path(x_start, x_end, y, step, delay=0.1):
    # ขยับไปยังตำแหน่งเริ่มต้นก่อนเริ่ม path
    send_position(x_start, y)
    time.sleep(1.0)
    while True:
        # วิ่งจากซ้ายไปขวา
        for x in range(x_start, x_end + 1, step):
            send_position(x, y)
            time.sleep(delay)
        # วิ่งจากขวากลับซ้าย (ไม่ซ้ำจุดปลาย)
        for x in range(x_end - step, x_start - 1, -step):
            send_position(x, y)
            time.sleep(delay)

def run_triangle_path(points=None, delay=0.3):
    # รับจุด path จาก argument หรือใช้ค่า default
    if points is None:
        points = [
            (-50, -120),
            (0, -60),
            (50, -120)
        ]
    # ขยับไปยังจุดเริ่มต้นก่อน
    send_position(*points[0])
    time.sleep(1.0)
    while True:
        for p in points:
            send_position(*p)
            time.sleep(delay)

# กำหนด serial port และ baudrate
ser = serial.Serial('COM11', 9600, timeout=1)  # เปลี่ยน 'COM3' ตามเครื่องของคุณ

run_triangle_path(points= [(-50, -150), (0, -50), (80, -150)], delay=0.2)

ser.close()