"""
IK Five-Bar Leg - Analytical Method
====================================

คำนวณ Inverse Kinematics แบบ Analytical สำหรับกลไก 5-Bar Linkage
รองรับ 4 Configurations (Elbow C/D: Up/Down)

Author: ธีรโชติ เมืองจำนงค์
Date: 17 พ.ย. 2568
"""

import numpy as np
import sys
import os

# เพิ่ม path สำหรับ import จาก parent directory
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# TODO: เพิ่ม import modules อื่นๆ ตามต้องการ
