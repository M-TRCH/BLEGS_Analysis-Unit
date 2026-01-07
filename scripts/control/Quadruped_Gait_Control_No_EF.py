"""
Quadruped Gait Control - Binary Protocol v1.2 (No EF Link Version)
Author: M-TRCH
Date: January 7, 2026

This script controls a quadruped robot (4 legs, 8 motors) WITHOUT EF link.
Joint E becomes the foot tip (end-effector).

Features:
- Automatic motor discovery and registration via COM port scanning
- Motor ID detection using PING command
- IK calculation WITHOUT EF link (E is the foot)
- Trot gait implementation
- Real-time visualization

Protocol: Binary Protocol v1.2 (see PROTOCOL.md)
"""

import numpy as np
import serial
import serial.tools.list_ports
import time
import threading
import struct
import traceback
import sys
import os
from enum import IntEnum
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import csv
from datetime import datetime

# Windows keyboard input
if sys.platform == 'win32':
    import msvcrt

# ============================================================================
# PROTOCOL CONSTANTS (v1.2)
# ============================================================================

HEADER_1 = 0xFE
HEADER_2 = 0xEE

class PacketType(IntEnum):
    """Packet type enumeration"""
    PKT_CMD_SET_GOAL = 0x01
    PKT_CMD_PING = 0x03
    PKT_CMD_EMERGENCY_STOP = 0x04
    PKT_FB_STATUS = 0x81
    PKT_FB_ERROR = 0x83

class ControlMode(IntEnum):
    """Control mode enumeration"""
    MODE_DIRECT_POSITION = 0x00
    MODE_SCURVE_PROFILE = 0x01

class StatusFlags(IntEnum):
    """Status flags bitmask"""
    STATUS_MOVING = (1 << 0)
    STATUS_ERROR = (1 << 1)
    STATUS_AT_GOAL = (1 << 2)
    STATUS_OVERHEAT = (1 << 3)
    STATUS_OVERCURRENT = (1 << 4)
    STATUS_ENCODER_ERROR = (1 << 5)
    STATUS_EMERGENCY_STOPPED = (1 << 6)

class ErrorCode(IntEnum):
    """Error code enumeration"""
    ERR_CRC_FAILED = 0x01
    ERR_INVALID_PACKET = 0x02
    ERR_TIMEOUT = 0x03
    ERR_UNKNOWN_COMMAND = 0x04
    ERR_INVALID_PAYLOAD = 0x05

# ============================================================================
# COMMUNICATION PARAMETERS
# ============================================================================

BAUD_RATE = 921600
SERIAL_TIMEOUT = 0.05  # 50ms timeout for discovery
FAST_TIMEOUT = 0.002   # 2ms timeout for high-speed operation
PING_RETRIES = 3       # Number of PING retries during discovery

# ============================================================================
# ROBOT CONFIGURATION (No EF Link)
# ============================================================================

# --- Robot Body Dimensions (mm) ---
BODY_LENGTH = 200.0   # Distance between front and rear hip axes
BODY_WIDTH = 170.0    # Distance between left and right hip axes

# --- Five-Bar Linkage Parameters (mm) ---
MOTOR_SPACING = 85.0  # Distance between Motor A and Motor B (horizontal)

# Link lengths (same for all 4 legs)
L_AC = 105.0  # Link 1 length (Motor A to joint C)
L_BD = 105.0  # Link 2 length (Motor B to joint D)
L_CE = 145.0  # Link 3 length (joint C to joint E)
L_DE = 145.0  # Link 4 length (joint D to joint E)

# NOTE: No L_EF link - Joint E is the foot!

# --- Motor Configuration ---
GEAR_RATIO = 8.0  # Motor shaft to output shaft gear ratio
MOTOR_INIT_ANGLE = -90.0  # Initial motor angle (degrees)

# --- Expected Motor IDs for Each Leg ---
EXPECTED_MOTOR_IDS = {
    'FL': {'A': 1, 'B': 2},  # Front Left
    'FR': {'A': 3, 'B': 4},  # Front Right
    'RL': {'A': 5, 'B': 6},  # Rear Left
    'RR': {'A': 7, 'B': 8}   # Rear Right
}

# --- Leg Motor Positions in Leg Frame (mm) ---
P_A_LEFT = np.array([-MOTOR_SPACING/2, 0.0])
P_B_LEFT = np.array([MOTOR_SPACING/2, 0.0])
P_A_RIGHT = np.array([MOTOR_SPACING/2, 0.0])
P_B_RIGHT = np.array([-MOTOR_SPACING/2, 0.0])

# --- Default Standing Pose ---
DEFAULT_STANCE_HEIGHT = -200.0  # mm (negative = down)
DEFAULT_STANCE_OFFSET_X = 0.0   # mm

# --- Motion Parameters ---
GAIT_LIFT_HEIGHT = 15.0    # mm
GAIT_STEP_FORWARD = 50.0   # mm

# Smooth Trot Parameters
SMOOTH_TROT_LIFT_HEIGHT = 15.0  # mm

# ============================================================================
# CONTROL PARAMETERS
# ============================================================================

CONTROL_MODE = ControlMode.MODE_DIRECT_POSITION
UPDATE_RATE = 50  # Hz (20ms per update)

TRAJECTORY_STEPS = 20  # Number of steps in one gait cycle

# Gait Types
DEFAULT_GAIT_TYPE = 'trot'
current_gait_type = DEFAULT_GAIT_TYPE

# Smooth Trot Parameters
SMOOTH_TROT_STEPS = 30
SMOOTH_TROT_STANCE_RATIO = 0.65

# --- Single Motor Mode ---
SINGLE_MOTOR_MODE = False
SINGLE_MOTOR_OSCILLATION = 30.0
SINGLE_MOTOR_PERIOD = 0.6
 
# --- Visualization Parameters ---
ENABLE_VISUALIZATION = True
PLOT_UPDATE_RATE = 10  # Hz

# --- Simulation Mode ---
SIMULATION_MODE = False  # Set to True to run without real motors

# --- Data Logging Parameters ---
ENABLE_DATA_LOGGING = True
LOG_DIRECTORY = "logs"

# ============================================================================
# GLOBAL VARIABLES
# ============================================================================

# Thread-safe locks
viz_lock = threading.Lock()
error_lock = threading.Lock()
control_lock = threading.Lock()

# Visualization state
plot_running = True

# Control state
gait_running = False
gait_paused = True

# Leg states
MOTOR_INIT_ANGLE_RAD = np.deg2rad(MOTOR_INIT_ANGLE)
leg_states = {
    'FR': {'target_angles': [MOTOR_INIT_ANGLE_RAD, MOTOR_INIT_ANGLE_RAD], 'actual_angles': [MOTOR_INIT_ANGLE_RAD, MOTOR_INIT_ANGLE_RAD], 'target_pos': [0.0, -200.0], 'phase': 0, 'color': 'red'},
    'FL': {'target_angles': [MOTOR_INIT_ANGLE_RAD, MOTOR_INIT_ANGLE_RAD], 'actual_angles': [MOTOR_INIT_ANGLE_RAD, MOTOR_INIT_ANGLE_RAD], 'target_pos': [0.0, -200.0], 'phase': 0, 'color': 'blue'},
    'RR': {'target_angles': [MOTOR_INIT_ANGLE_RAD, MOTOR_INIT_ANGLE_RAD], 'actual_angles': [MOTOR_INIT_ANGLE_RAD, MOTOR_INIT_ANGLE_RAD], 'target_pos': [0.0, -200.0], 'phase': 0, 'color': 'orange'},
    'RL': {'target_angles': [MOTOR_INIT_ANGLE_RAD, MOTOR_INIT_ANGLE_RAD], 'actual_angles': [MOTOR_INIT_ANGLE_RAD, MOTOR_INIT_ANGLE_RAD], 'target_pos': [0.0, -200.0], 'phase': 0, 'color': 'green'}
}

# Motor registry
motor_registry = {}
leg_motors = {}

# Error tracking
error_stats = {}

# Single motor mode state
single_motor_controller = None
single_motor_start_time = None

# Data logging
log_file = None
log_writer = None
log_lock = threading.Lock()
log_start_time = 0.0
log_record_count = 0

# ============================================================================
# PROTOCOL FUNCTIONS
# ============================================================================

def calculate_crc16(data: bytes) -> int:
    """Calculate CRC-16-IBM for data buffer"""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF

def build_packet(pkt_type: int, payload: bytes = b'') -> bytes:
    """Build a complete binary packet with CRC"""
    header = bytes([HEADER_1, HEADER_2])
    type_byte = bytes([pkt_type])
    payload_len = bytes([len(payload)])
    
    crc_data = type_byte + payload_len + payload
    crc = calculate_crc16(crc_data)
    crc_bytes = struct.pack('<H', crc)
    
    return header + type_byte + payload_len + payload + crc_bytes

# ============================================================================
# MOTOR CONTROLLER CLASS
# ============================================================================

class BinaryMotorController:
    """High-speed motor controller using binary protocol v1.2"""
    
    def __init__(self, port, motor_id=None):
        self.port = port
        self.motor_id = motor_id
        self.serial = None
        self.is_connected = False
        self.current_setpoint = 0.0
        self.current_position = 0.0
        self.current_current = 0
        self.current_flags = 0
        self.lock = threading.Lock()
        self.stats_tx_count = 0
        self.stats_rx_count = 0
        self.stats_errors = 0
    
    def connect(self, timeout=SERIAL_TIMEOUT):
        """Connect to motor via serial port"""
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=BAUD_RATE,
                timeout=timeout,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            )
            time.sleep(0.1)
            
            self.serial.reset_input_buffer()
            self.serial.reset_output_buffer()
            
            self.is_connected = True
            return True
            
        except Exception as e:
            self.is_connected = False
            return False
    
    def disconnect(self):
        """Disconnect from motor"""
        if self.serial and self.is_connected:
            try:
                self.serial.close()
            except:
                pass
            self.is_connected = False
    
    def set_timeout(self, timeout):
        """Update serial timeout"""
        if self.serial and self.is_connected:
            self.serial.timeout = timeout
    
    def send_ping(self) -> dict:
        """Send PING command and wait for response"""
        if not self.is_connected:
            return None
        
        try:
            self.serial.reset_input_buffer()
            
            packet = build_packet(PacketType.PKT_CMD_PING)
            self.serial.write(packet)
            self.stats_tx_count += 1
            
            response = self._read_packet()
            if response:
                pkt_type, payload = response
                if pkt_type == PacketType.PKT_FB_STATUS and len(payload) >= 8:
                    motor_id = payload[0]
                    position_raw = struct.unpack('<i', payload[1:5])[0]
                    current_raw = struct.unpack('<h', payload[5:7])[0]
                    status_flags = payload[7]
                    
                    self.motor_id = motor_id
                    self.current_position = (position_raw / 100.0) / GEAR_RATIO
                    self.current_current = current_raw
                    self.current_flags = status_flags
                    
                    return {
                        'motor_id': motor_id,
                        'position': self.current_position,
                        'current': current_raw,
                        'flags': status_flags
                    }
            return None
            
        except Exception as e:
            return None
    
    def _read_packet(self):
        """Read and parse incoming packet"""
        try:
            header = self.serial.read(2)
            if len(header) != 2 or header[0] != HEADER_1 or header[1] != HEADER_2:
                return None
            
            pkt_type = self.serial.read(1)[0]
            payload_len = self.serial.read(1)[0]
            
            payload = self.serial.read(payload_len) if payload_len > 0 else b''
            crc_bytes = self.serial.read(2)
            
            if len(crc_bytes) != 2:
                return None
            
            received_crc = struct.unpack('<H', crc_bytes)[0]
            calculated_crc = calculate_crc16(bytes([pkt_type, payload_len]) + payload)
            
            if received_crc != calculated_crc:
                return None
            
            self.stats_rx_count += 1
            return pkt_type, payload
            
        except Exception as e:
            return None
    
    def start_motor(self) -> bool:
        """Start motor using PING command"""
        for retry in range(PING_RETRIES):
            result = self.send_ping()
            if result:
                print(f"  ✅ Motor ID {result['motor_id']} started on {self.port}")
                time.sleep(0.1)
                return True
            time.sleep(0.1)
        return False
    
    def set_position_direct(self, angle_deg: float) -> bool:
        """Set target position using direct mode"""
        if not self.is_connected:
            return False
        
        try:
            motor_angle = angle_deg * GEAR_RATIO
            
            with self.lock:
                mode = bytes([ControlMode.MODE_DIRECT_POSITION])
                target_pos = struct.pack('<i', int(motor_angle * 100))
                payload = mode + target_pos
                
                packet = build_packet(PacketType.PKT_CMD_SET_GOAL, payload)
                self.serial.write(packet)
                self.serial.flush()
                self.stats_tx_count += 1
                self.current_setpoint = angle_deg
            
            return True
            
        except Exception as e:
            with error_lock:
                motor_id = self.motor_id if self.motor_id else "Unknown"
                error_stats[motor_id] = error_stats.get(motor_id, 0) + 1
            return False
    
    def send_emergency_stop(self) -> bool:
        """Send emergency stop command"""
        if not self.is_connected:
            return False
        
        try:
            packet = build_packet(PacketType.PKT_CMD_EMERGENCY_STOP)
            self.serial.write(packet)
            self.serial.flush()
            return True
        except:
            return False

# ============================================================================
# MOTOR DISCOVERY AND REGISTRATION
# ============================================================================

def discover_motors() -> dict:
    """Scan COM ports and discover motors"""
    print("\n" + "="*70)
    print("  🔍 MOTOR DISCOVERY")
    print("="*70)
    
    ports = []
    for port in serial.tools.list_ports.comports():
        ports.append({'device': port.device, 'description': port.description})
    
    if not ports:
        print("  ❌ No COM ports found!")
        print("\n  💡 Would you like to run in SIMULATION mode? (no real motors)")
        response = input("  Run simulation? [y/N]: ").strip().lower()
        if response == 'y':
            global SIMULATION_MODE, ENABLE_VISUALIZATION
            SIMULATION_MODE = True
            ENABLE_VISUALIZATION = True
            print("  ✅ Simulation mode enabled")
        return {}
    
    print(f"\n  Found {len(ports)} COM port(s):")
    for port in ports:
        print(f"    • {port['device']}: {port['description']}")
    
    discovered = {}
    
    print(f"\n  📡 Scanning for motors (timeout={SERIAL_TIMEOUT*1000:.0f}ms, retries={PING_RETRIES})...")
    
    for port_info in ports:
        port = port_info['device']
        print(f"\n    Checking {port}...")
        
        controller = BinaryMotorController(port)
        
        if not controller.connect(timeout=SERIAL_TIMEOUT):
            print(f"      ❌ Failed to open {port}")
            continue
        
        motor_info = None
        for attempt in range(PING_RETRIES):
            motor_info = controller.send_ping()
            if motor_info:
                break
            time.sleep(0.05)
        
        if motor_info:
            motor_id = motor_info['motor_id']
            print(f"      ✅ Motor ID {motor_id} found!")
            print(f"         Position: {motor_info['position']:.2f}°")
            print(f"         Current:  {motor_info['current']} mA")
            print(f"         Flags:    0x{motor_info['flags']:02X}")
            
            discovered[motor_id] = controller
        else:
            print(f"      ⚠️  No response (no motor on this port)")
            controller.disconnect()
    
    print(f"\n  📊 Discovery complete: {len(discovered)} motor(s) found")
    return discovered

def register_leg_motors(discovered_motors: dict) -> bool:
    """Register discovered motors to leg assignments"""
    global leg_motors, motor_registry
    
    print("\n" + "="*70)
    print("  🔗 MOTOR REGISTRATION")
    print("="*70)
    
    motor_registry = discovered_motors
    leg_motors = {}
    
    all_assigned = True
    missing_motors = []
    
    for leg_id, motor_ids in EXPECTED_MOTOR_IDS.items():
        motor_a_id = motor_ids['A']
        motor_b_id = motor_ids['B']
        
        motor_a = discovered_motors.get(motor_a_id)
        motor_b = discovered_motors.get(motor_b_id)
        
        if motor_a and motor_b:
            leg_motors[leg_id] = {
                'A': motor_a,
                'B': motor_b,
                'motor_a_id': motor_a_id,
                'motor_b_id': motor_b_id
            }
            print(f"    ✅ {leg_id}: Motor A (ID {motor_a_id}) + Motor B (ID {motor_b_id})")
        else:
            all_assigned = False
            if not motor_a:
                missing_motors.append(f"{leg_id} Motor A (ID {motor_a_id})")
            if not motor_b:
                missing_motors.append(f"{leg_id} Motor B (ID {motor_b_id})")
            print(f"    ❌ {leg_id}: INCOMPLETE - Missing motor(s)")
    
    if missing_motors:
        print(f"\n  ⚠️  Missing motors:")
        for m in missing_motors:
            print(f"      • {m}")
    
    return all_assigned

def start_all_motors() -> bool:
    """Start all registered motors using Binary Protocol PING"""
    print("\n" + "="*70)
    print("  🚀 STARTING MOTORS")
    print("="*70)
    
    success_count = 0
    total_count = len(motor_registry)
    
    for motor_id, controller in motor_registry.items():
        controller.set_timeout(FAST_TIMEOUT)
        
        result = controller.send_ping()
        if result:
            success_count += 1
        else:
            print(f"  ❌ Failed to start Motor ID {motor_id}")
    
    print(f"\n  📊 Started {success_count}/{total_count} motors")
    
    if success_count > 0:
        print("  ⏳ Waiting for motor initialization...")
        time.sleep(2.0)
    
    return success_count == total_count

# ============================================================================
# KINEMATICS FUNCTIONS (No EF Link)
# ============================================================================

def get_motor_positions(leg_id):
    """Get motor positions based on leg side"""
    if leg_id in ['FL', 'RL']:
        return P_A_LEFT.copy(), P_B_LEFT.copy()
    else:  # FR, RR
        return P_A_RIGHT.copy(), P_B_RIGHT.copy()

def solve_circle_intersection(center1, radius1, center2, radius2, choose_lower=True):
    """Find intersection of two circles"""
    V_12 = center2 - center1
    d = np.linalg.norm(V_12)
    
    if d > (radius1 + radius2) or d < abs(radius1 - radius2) or d == 0:
        return np.array([np.nan, np.nan])
    
    a = (radius1**2 - radius2**2 + d**2) / (2 * d)
    h_squared = radius1**2 - a**2
    
    if h_squared < 0:
        return np.array([np.nan, np.nan])
    
    h = np.sqrt(h_squared)
    v_d = V_12 / d
    v_perp = np.array([-v_d[1], v_d[0]])
    
    P_intersection_1 = center1 + a * v_d + h * v_perp
    P_intersection_2 = center1 + a * v_d - h * v_perp
    
    if choose_lower:
        return P_intersection_2 if P_intersection_2[1] < P_intersection_1[1] else P_intersection_1
    else:
        return P_intersection_1 if P_intersection_1[1] > P_intersection_2[1] else P_intersection_2

def calculate_ik_no_ef(P_E_target, P_A, P_B, elbow_C_down=True, elbow_D_down=True):
    """
    Calculate Inverse Kinematics WITHOUT EF link (E is the foot)
    
    Args:
        P_E_target: Target foot position [x, y] in leg frame
        P_A: Motor A position [x, y]
        P_B: Motor B position [x, y]
        elbow_C_down: True to choose lower elbow position for joint C
        elbow_D_down: True to choose lower elbow position for joint D
        
    Returns:
        numpy array [theta_A, theta_B] in radians, or [nan, nan] if no solution
    """
    # Find joint C position
    P_C = solve_circle_intersection(P_A, L_AC, P_E_target, L_CE, elbow_C_down)
    
    if np.isnan(P_C).any():
        return np.array([np.nan, np.nan])
    
    # Find joint D position
    P_D = solve_circle_intersection(P_B, L_BD, P_E_target, L_DE, elbow_D_down)
    
    if np.isnan(P_D).any():
        return np.array([np.nan, np.nan])
    
    # Calculate motor angles
    V_AC = P_C - P_A
    V_BD = P_D - P_B
    
    theta_A = np.arctan2(V_AC[1], V_AC[0])
    theta_B = np.arctan2(V_BD[1], V_BD[0])
    
    return np.array([theta_A, theta_B])

def calculate_fk_no_ef(theta_A, theta_B, P_A, P_B):
    """
    Calculate forward kinematics positions WITHOUT EF link
    
    Returns:
        P_C, P_D, P_E (foot position)
    """
    # Calculate P_C and P_D from motor angles
    P_C = P_A + np.array([L_AC * np.cos(theta_A), L_AC * np.sin(theta_A)])
    P_D = P_B + np.array([L_BD * np.cos(theta_B), L_BD * np.sin(theta_B)])
    
    # Calculate P_E using circle intersection between C and D
    V_CD = P_D - P_C
    d = np.linalg.norm(V_CD)
    
    if d > 0 and d <= (L_CE + L_DE) and d >= abs(L_CE - L_DE):
        a = (L_CE**2 - L_DE**2 + d**2) / (2 * d)
        h_squared = L_CE**2 - a**2
        
        if h_squared >= 0:
            h = np.sqrt(h_squared)
            v_d = V_CD / d
            v_perp = np.array([-v_d[1], v_d[0]])
            
            # Try both configurations and choose the one with lower y (foot down)
            P_E1 = P_C + a * v_d + h * v_perp
            P_E2 = P_C + a * v_d - h * v_perp
            
            # Choose configuration with lower foot position
            if P_E1[1] < P_E2[1]:
                P_E = P_E1
            else:
                P_E = P_E2
            
            return P_C, P_D, P_E
    
    return None, None, None

# ============================================================================
# TRAJECTORY GENERATION
# ============================================================================

def generate_elliptical_trajectory(step_forward, lift_height, num_steps, stance_ratio=0.5, 
                                   home_x=0.0, home_y=DEFAULT_STANCE_HEIGHT, 
                                   reverse=False, mirror_x=False):
    """Generate elliptical foot trajectory"""
    trajectory = []
    
    swing_steps = int(num_steps * (1.0 - stance_ratio))
    stance_steps = num_steps - swing_steps
    
    direction = -1 if reverse else 1
    
    for i in range(num_steps):
        if i < swing_steps:
            phase_progress = i / swing_steps
            t = np.pi + np.pi * phase_progress  # π to 2π
        else:
            stance_index = i - swing_steps
            phase_progress = stance_index / stance_steps
            t = np.pi * phase_progress  # 0 to π
        
        px = direction * (-step_forward * np.cos(t))
        if mirror_x:
            px = -px
        
        if i < swing_steps:  # Swing phase
            py = home_y + lift_height * abs(np.sin(t))
        else:  # Stance phase
            py = home_y
        
        trajectory.append((px + home_x, py))
    
    return trajectory

def get_gait_phase_offset(leg_id):
    """Get phase offset for each leg based on current gait type"""
    global current_gait_type
    
    if current_gait_type in ['trot', 'smooth_trot', 'backward_trot']:
        offsets = {'FR': 0.0, 'FL': 0.5, 'RR': 0.5, 'RL': 0.0}
        return offsets[leg_id]
    elif current_gait_type in ['walk', 'crawl']:
        offsets = {'FR': 0.0, 'RR': 0.25, 'FL': 0.5, 'RL': 0.75}
        return offsets[leg_id]
    elif current_gait_type == 'stand':
        return 0.0
    
    return 0.0

# ============================================================================
# CONTROL FUNCTIONS
# ============================================================================

def toggle_gait_control():
    """Toggle gait control on/off"""
    global gait_running, gait_paused
    with control_lock:
        if gait_paused:
            gait_running = True
            gait_paused = False
            print("\n▶️  Gait control STARTED!")
        else:
            gait_running = False
            gait_paused = True
            print("\n⏸️  Gait control PAUSED!")

def emergency_stop_all():
    """Send emergency stop to all motors"""
    print("\n⚠️  EMERGENCY STOP - ALL MOTORS!")
    for motor_id, controller in motor_registry.items():
        controller.send_emergency_stop()

def initialize_data_logging():
    """Initialize CSV log file for motor feedback data"""
    global log_file, log_writer
    
    if not ENABLE_DATA_LOGGING:
        return
    
    if not os.path.exists(LOG_DIRECTORY):
        os.makedirs(LOG_DIRECTORY)
        print(f"  📁 Created log directory: {LOG_DIRECTORY}")
    
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_filename = os.path.join(LOG_DIRECTORY, f"motor_feedback_no_ef_{timestamp}.csv")
    
    try:
        log_file = open(log_filename, 'w', newline='', buffering=1)
        log_writer = csv.writer(log_file)
        
        log_writer.writerow([
            'timestamp', 'elapsed_ms', 'motor_id', 'setpoint_deg',
            'position_deg', 'error_deg', 'current_mA', 'flags_hex'
        ])
        log_file.flush()
        
        print(f"  📝 Data logging initialized: {log_filename}")
        return True
        
    except Exception as e:
        print(f"  ❌ Failed to initialize data logging: {e}")
        return False

def close_data_logging():
    """Close log file"""
    global log_file, log_writer, log_record_count
    
    if not ENABLE_DATA_LOGGING or log_file is None:
        return
    
    try:
        with log_lock:
            if log_file:
                log_file.flush()
                log_file.close()
                print(f"  📝 Data log file closed ({log_record_count} records written)")
                log_file = None
                log_writer = None
    except Exception as e:
        print(f"  ⚠️ Error closing log: {e}")

# ============================================================================
# VISUALIZATION
# ============================================================================

def create_leg_subplot(ax, leg_id, leg_name):
    """Create subplot for one leg"""
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    ax.set_xlim(-150, 150)
    ax.set_ylim(-300, 50)
    ax.set_xlabel('Y (mm)', fontsize=9)
    ax.set_ylabel('X (mm)', fontsize=9)
    ax.set_title(f'{leg_id} - {leg_name} (No EF)', fontsize=10, weight='bold')
    
    P_A, P_B = get_motor_positions(leg_id)
    
    # Draw motor positions
    ax.plot(P_A[0], P_A[1], 'o', color='darkblue', markersize=10, label='Motor A', zorder=5)
    ax.plot(P_B[0], P_B[1], 'o', color='darkred', markersize=10, label='Motor B', zorder=5)
    
    color = leg_states[leg_id]['color']
    
    # Links
    link1 = ax.plot([], [], '-', color='darkblue', linewidth=3, label='AC', zorder=4)[0]
    link2 = ax.plot([], [], '-', color='darkred', linewidth=3, label='BD', zorder=4)[0]
    link3 = ax.plot([], [], '--', color='orange', linewidth=2, label='CE', zorder=3)[0]
    link4 = ax.plot([], [], '--', color='cyan', linewidth=2, label='DE', zorder=3)[0]
    
    # Joints
    joint_c = ax.plot([], [], 'ro', markersize=7, zorder=5)[0]
    joint_d = ax.plot([], [], 'bo', markersize=7, zorder=5)[0]
    joint_e_foot = ax.plot([], [], '*', color='green', markersize=15, label='Foot (E)', zorder=6)[0]
    
    info_text = ax.text(0.02, 0.98, '', transform=ax.transAxes,
                        fontsize=7, verticalalignment='top', family='monospace',
                        bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.9))
    
    ax.legend(loc='lower right', fontsize=7, framealpha=0.9)
    
    return {
        'links': [link1, link2, link3, link4],
        'joints': [joint_c, joint_d, joint_e_foot],
        'info_text': info_text
    }

def update_leg_plot(leg_id, plot_elements):
    """Update one leg's plot"""
    with viz_lock:
        theta_A, theta_B = leg_states[leg_id]['target_angles']
        target_x, target_y = leg_states[leg_id]['target_pos']
    
    P_A, P_B = get_motor_positions(leg_id)
    
    # Calculate FK
    P_C, P_D, P_E = calculate_fk_no_ef(theta_A, theta_B, P_A, P_B)
    
    if P_C is not None:
        # Update links
        plot_elements['links'][0].set_data([P_A[0], P_C[0]], [P_A[1], P_C[1]])
        plot_elements['links'][1].set_data([P_B[0], P_D[0]], [P_B[1], P_D[1]])
        plot_elements['links'][2].set_data([P_C[0], P_E[0]], [P_C[1], P_E[1]])
        plot_elements['links'][3].set_data([P_D[0], P_E[0]], [P_D[1], P_E[1]])
        
        # Update joints
        plot_elements['joints'][0].set_data([P_C[0]], [P_C[1]])
        plot_elements['joints'][1].set_data([P_D[0]], [P_D[1]])
        plot_elements['joints'][2].set_data([P_E[0]], [P_E[1]])
        
        # Update info text
        mode_text = "SIMULATION" if SIMULATION_MODE else "REAL"
        plot_elements['info_text'].set_text(
            f'Mode: {mode_text}\n'
            f'Motor A: {np.rad2deg(theta_A):+6.1f}°\n'
            f'Motor B: {np.rad2deg(theta_B):+6.1f}°\n'
            f'Foot (E): ({P_E[0]:.0f},{P_E[1]:.0f})'
        )

def visualization_thread():
    """Thread function for real-time visualization"""
    global plot_running
    
    fig = plt.figure(figsize=(14, 10))
    mode_text = "SIMULATION MODE" if SIMULATION_MODE else "REAL MODE"
    fig.suptitle(f'Quadruped Gait Control - {current_gait_type.upper()} ({mode_text})', 
                 fontsize=14, weight='bold')
    
    # Create 2x2 grid for 4 legs
    ax_FR = plt.subplot(2, 2, 1)
    ax_FL = plt.subplot(2, 2, 2)
    ax_RR = plt.subplot(2, 2, 3)
    ax_RL = plt.subplot(2, 2, 4)
    
    # Create plot elements for each leg
    plot_elements = {
        'FR': create_leg_subplot(ax_FR, 'FR', 'Front Right'),
        'FL': create_leg_subplot(ax_FL, 'FL', 'Front Left'),
        'RR': create_leg_subplot(ax_RR, 'RR', 'Rear Right'),
        'RL': create_leg_subplot(ax_RL, 'RL', 'Rear Left')
    }
    
    # Add control instructions
    control_text = 'Controls: [SPACE] Start/Stop'
    if SIMULATION_MODE:
        control_text += ' | SIMULATION MODE - No real motors'
    fig.text(0.5, 0.02, control_text, 
             ha='center', fontsize=10, family='monospace',
             bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.8))
    
    def update_plot(frame):
        if not plot_running:
            return []
        
        artists = []
        for leg_id in ['FR', 'FL', 'RR', 'RL']:
            update_leg_plot(leg_id, plot_elements[leg_id])
            artists.extend(plot_elements[leg_id]['links'])
            artists.extend(plot_elements[leg_id]['joints'])
            artists.append(plot_elements[leg_id]['info_text'])
        
        return artists
    
    def on_key_press(event):
        if event.key == ' ':
            toggle_gait_control()
    
    fig.canvas.mpl_connect('key_press_event', on_key_press)
    
    anim = FuncAnimation(fig, update_plot, interval=int(1000/PLOT_UPDATE_RATE), 
                        blit=True, cache_frame_data=False)
    
    plt.tight_layout(rect=[0, 0.03, 1, 0.98])
    plt.show()
    
    plot_running = False

# ============================================================================
# GAIT CONTROL LOOP
# ============================================================================

def gait_control_loop():
    """Main gait control loop"""
    global gait_running, gait_paused, leg_states, log_start_time
    
    print("\n" + "="*70)
    print("  🚶 GAIT CONTROL LOOP (No EF Link)")
    print("="*70)
    print(f"  Mode: {current_gait_type.upper()}")
    print(f"  Update Rate: {UPDATE_RATE} Hz")
    print(f"  Control Mode: DIRECT_POSITION")
    print(f"  NOTE: Joint E is the foot (no EF link)")
    print("="*70)
    
    # Generate trajectories
    trajectories = {}
    for leg_id in ['FR', 'FL', 'RR', 'RL']:
        mirror_x = (leg_id in ['FR', 'RR'])
        reverse = (current_gait_type == 'backward_trot')
        
        trajectories[leg_id] = generate_elliptical_trajectory(
            step_forward=GAIT_STEP_FORWARD,
            lift_height=GAIT_LIFT_HEIGHT,
            num_steps=TRAJECTORY_STEPS,
            stance_ratio=0.5,
            reverse=reverse,
            mirror_x=mirror_x
        )
    
    # Initialize previous solutions for smooth IK
    prev_solutions = {}
    for leg_id in ['FR', 'FL', 'RR', 'RL']:
        home_pos = np.array([DEFAULT_STANCE_OFFSET_X, DEFAULT_STANCE_HEIGHT])
        P_A, P_B = get_motor_positions(leg_id)
        home_angles = calculate_ik_no_ef(home_pos, P_A, P_B, elbow_C_down=True, elbow_D_down=True)
        
        if not np.isnan(home_angles).any():
            prev_solutions[leg_id] = home_angles
        else:
            prev_solutions[leg_id] = None
    
    log_start_time = time.time()
    frame = 0
    cycle_count = 0
    
    print("\n⏸️  Gait ready - Press SPACE to start")
    
    while gait_running:
        if gait_paused:
            time.sleep(0.05)
            continue
        
        loop_start = time.perf_counter()
        
        # Update each leg
        for leg_id in ['FR', 'FL', 'RR', 'RL']:
            # In simulation mode, skip motor check
            if not SIMULATION_MODE and leg_id not in leg_motors:
                continue
            
            # Calculate current phase
            phase_offset = get_gait_phase_offset(leg_id)
            current_phase = (frame + int(phase_offset * TRAJECTORY_STEPS)) % TRAJECTORY_STEPS
            
            # Get target position from trajectory
            px, py = trajectories[leg_id][current_phase]
            
            # Calculate IK - try all configurations
            configs = [
                (True, True),   # C=Down, D=Down
                (True, False),  # C=Down, D=Up
                (False, True),  # C=Up, D=Down
                (False, False)  # C=Up, D=Up
            ]
            
            best_solution = None
            best_distance = float('inf')
            P_A, P_B = get_motor_positions(leg_id)
            
            for elbow_C, elbow_D in configs:
                solution = calculate_ik_no_ef(
                    np.array([px, py]),
                    P_A, P_B,
                    elbow_C_down=elbow_C,
                    elbow_D_down=elbow_D
                )
                
                if not np.isnan(solution).any():
                    if prev_solutions[leg_id] is None:
                        if elbow_C and elbow_D:
                            best_solution = solution
                            break
                    else:
                        angle_diff = np.abs(solution - prev_solutions[leg_id])
                        angle_diff = np.minimum(angle_diff, 2*np.pi - angle_diff)
                        distance = np.sum(angle_diff)
                        
                        if distance < best_distance:
                            best_distance = distance
                            best_solution = solution
            
            if best_solution is not None and not np.isnan(best_solution).any():
                prev_solutions[leg_id] = best_solution
                theta_A, theta_B = best_solution
                
                # Send to motors (only in real mode)
                if not SIMULATION_MODE:
                    motor_a = leg_motors[leg_id]['A']
                    motor_b = leg_motors[leg_id]['B']
                    
                    motor_a.set_position_direct(np.rad2deg(theta_A))
                    motor_b.set_position_direct(np.rad2deg(theta_B))
                
                # Update visualization state
                with viz_lock:
                    leg_states[leg_id]['target_angles'] = [theta_A, theta_B]
                    leg_states[leg_id]['target_pos'] = [px, py]
                    leg_states[leg_id]['phase'] = current_phase
        
        # Update frame counter
        frame = (frame + 1) % TRAJECTORY_STEPS
        
        if frame == 0:
            cycle_count += 1
            if cycle_count % 10 == 0:
                print(f"  Cycle #{cycle_count}")
        
        # Timing control
        elapsed = time.perf_counter() - loop_start
        sleep_time = max(0, (1.0 / UPDATE_RATE) - elapsed)
        if sleep_time > 0:
            time.sleep(sleep_time)
    
    print("\n⏹️  Gait control loop stopped")

# ============================================================================
# KEYBOARD INPUT
# ============================================================================

def keyboard_input_thread():
    """Thread for handling keyboard input"""
    global gait_running
    
    print("\n" + "="*70)
    print("  ⌨️  KEYBOARD CONTROLS")
    print("="*70)
    if SIMULATION_MODE:
        print("  Use visualization window:")
        print("  [SPACE]  - Start/Pause gait (press in plot window)")
        print("  [Q]      - Quit (type here and press Enter)")
    else:
        print("  [SPACE]  - Start/Pause gait")
        print("  [E]      - Emergency stop all motors")
        print("  [Q]      - Quit program")
    print("="*70)
    
    if SIMULATION_MODE:
        # In simulation mode, wait for 'q' input
        while gait_running:
            try:
                cmd = input()
                if cmd.lower() == 'q':
                    print("\n👋 Quitting...")
                    gait_running = False
                    break
            except:
                time.sleep(0.1)
    else:
        # Real mode with keyboard monitoring
        while gait_running:
            if sys.platform == 'win32' and msvcrt.kbhit():
                key = msvcrt.getch()
                
                if key == b' ':
                    toggle_gait_control()
                elif key.lower() == b'e':
                    emergency_stop_all()
                elif key.lower() == b'q':
                    print("\n👋 Quitting...")
                    gait_running = False
                    break
            
            time.sleep(0.05)

# ============================================================================
# MAIN FUNCTION
# ============================================================================

def main():
    global gait_running, gait_paused, motor_registry, leg_motors
    
    print("="*70)
    print("  BLEGS Quadruped Gait Control - No EF Link Version")
    print("="*70)
    print(f"  Protocol: Binary Protocol v1.2")
    print(f"  Baud Rate: {BAUD_RATE}")
    print(f"  Link Configuration: AC={L_AC}mm, BD={L_BD}mm, CE={L_CE}mm, DE={L_DE}mm")
    print(f"  NOTE: Joint E is the FOOT (no EF link)")
    print("="*70)
    
    # Discover motors
    discovered = discover_motors()
    
    if len(discovered) == 0 and not SIMULATION_MODE:
        print("\n❌ No motors discovered and simulation not enabled. Exiting.")
        return
    
    if not SIMULATION_MODE:
        # Register motors to legs
        all_assigned = register_leg_motors(discovered)
        
        if not all_assigned:
            print("\n⚠️  Warning: Not all motors assigned to legs")
            response = input("  Continue anyway? [y/N]: ")
            if response.lower() != 'y':
                print("\n👋 Exiting...")
                return
        
        # Start motors
        if not start_all_motors():
            print("\n❌ Failed to start some motors. Exiting.")
            return
    else:
        print("\n" + "="*70)
        print("  🎮 SIMULATION MODE ACTIVATED")
        print("="*70)
        print("  Running without real motors")
        print("  Visualization will show leg movements")
        print("="*70)
    
    # Initialize data logging
    if ENABLE_DATA_LOGGING:
        initialize_data_logging()
    
    # Start gait control
    gait_running = True
    
    gait_thread = threading.Thread(target=gait_control_loop, daemon=True)
    gait_thread.start()
    
    # Start visualization if enabled
    if ENABLE_VISUALIZATION:
        viz_thread = threading.Thread(target=visualization_thread, daemon=True)
        viz_thread.start()
    
    # Start keyboard input handler
    try:
        keyboard_input_thread()
    except KeyboardInterrupt:
        print("\n\n⏹️  Interrupted by user")
    
    # Cleanup
    gait_running = False
    plot_running = False
    time.sleep(0.5)
    
    print("\n" + "="*70)
    print("  🛑 SHUTTING DOWN")
    print("="*70)
    
    # Close data logging
    if ENABLE_DATA_LOGGING and not SIMULATION_MODE:
        close_data_logging()
    
    # Disconnect motors (only in real mode)
    if not SIMULATION_MODE:
        print("  Disconnecting motors...")
        for motor_id, controller in motor_registry.items():
            controller.disconnect()
    
    if ENABLE_VISUALIZATION:
        print("  Closing visualization...")
        plt.close('all')
    
    print("\n✅ Program terminated successfully")
    print("="*70)

if __name__ == "__main__":
    main()
