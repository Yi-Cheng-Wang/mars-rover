import struct

# ==========================================
# 1. Basic Settings & System Configuration
# ==========================================
class CAN_CONFIG:
    PORT = 'COM14'
    BAUDRATE = 2000000
    TIMEOUT = 0.1
    WHEEL_CAN_IDS = [0x01, 0x02, 0x03, 0x04]
    INQUIRY_FREQUENCY = 0.01   # Seconds
    INQUIRY_DURATION = 10.0    # Seconds

class MOTOR_LIMITS:
    DEFAULT_SPEED = [0.0, 0.0, 0.0, 0.0]
    MAX_SPEED = [5000.0, 5000.0, 5000.0, 5000.0]
    MIN_SPEED = [0.0, 0.0, 0.0, 0.0]
    DEFAULT_POS = [0.0, 0.0, 0.0, 0.0]
    MOVE_STEP_VEL = 500.0

# ==========================================
# 2. Briter Motor Control Protocol Class
# ==========================================
class BriterMotorProtocol:
    """
    Briter Motor Communication Protocol Class
    Applicable for communication between USB-to-CAN modules and motor drivers.
    """

    # --- Basic Communication Constants ---
    HEADER = 0xAA
    FOOTER = 0x55
    INFO_BASE = 0xC0  # Base Info Byte (Standard Frame)

    # --- Control Commands (DATA[0] / Byte 4) ---
    CMD_HEARTBEAT = 0x00         # Heartbeat
    CMD_SET_CURRENT = 0x01       # Set Current (Unit: 10mA)
    CMD_SET_SPEED = 0x02         # Set Speed (Unit: erpm)
    CMD_SET_DUTY = 0x03          # Set Duty Cycle (-1000 to 1000)
    CMD_SET_POS_ABS = 0x04       # Set Absolute Position (Unit: 0.01 deg)
    CMD_SET_POS_REL_TRG = 0x05    # Set Relative Increment (Target-based)
    CMD_SET_POS_REL_CUR = 0x06    # Set Relative Increment (Current-based)
    CMD_SET_POS_CUR = 0x07        # Define current position as a specific value
    CMD_SET_BRAKE_CUR = 0x08      # Set Braking Current
    CMD_SET_HANDBRAKE_CUR = 0x09  # Set Handbrake Current
    CMD_SET_ACCEL_SPEED = 0x0A    # Set Speed Loop Acceleration
    CMD_SET_TRAJ_VEL_MAX = 0x0B   # Set Trajectory Max Velocity
    CMD_SET_TRAJ_ACC_MAX = 0x0C   # Set Trajectory Max Acceleration
    CMD_SET_TRAJ_DEC_MAX = 0x0D   # Set Trajectory Max Deceleration
    CMD_SWITCH_CONFIG = 0x0E      # Switch Configuration Table
    CMD_INQUIRY = 0x0F            # Inquiry/Query Command
    CMD_SET_DECEL_SPEED = 0x10    # Set Speed Loop Deceleration
    CMD_HOME_START = 0x11         # Start Homing
    CMD_HOME_STOP = 0x12          # Stop Homing
    CMD_HOME_STATUS = 0x13        # Query Homing Status
    CMD_SET_MAX_CURRENT = 0x14    # Set Max Current for Closed-Loop

    # --- Inquiry Sub-commands (DATA[1] / Byte 5) ---
    QUERY_FAULT = 0x00            # Fault Information
    QUERY_SPEED = 0x01            # Current Speed (erpm)
    QUERY_DUTY = 0x02             # Duty Cycle
    QUERY_POWER = 0x03            # Power (W)
    QUERY_VOLTAGE = 0x04          # Voltage (V)
    QUERY_CURRENT_MOTOR = 0x05    # Motor Current (10mA)
    QUERY_CURRENT_BUS = 0x06      # Bus Current (10mA)
    QUERY_TEMP = 0x07             # Temperature (℃)
    QUERY_POS_ABS = 0x08          # Accumulated Absolute Position (0.01 deg)
    QUERY_POS_CIRCLE = 0x09       # Single-turn Angle (0.01 deg)
    QUERY_ENCODER_MODE = 0x0A     # Encoder Mode
    QUERY_FAULT_HISTORY = 0x0B    # Fault History
    QUERY_IO_STATUS = 0x0C        # Current IO Status

    @staticmethod
    def get_init_frame():
        """Get the initialization frame for the USB-to-CAN module"""
        frame = [
            0xaa, 0x55, 0x12, 0x03, 0x01, 
            0x00, 0x00, 0x00, 0x00, 
            0x00, 0x00, 0x00, 0x00, 
            0x00, 0x01, 0x00, 0x00, 0x00, 0x00
        ]
        checksum = sum(frame[2:19]) & 0xFF
        frame.append(checksum)
        return bytearray(frame)

    @classmethod
    def pack_command_frame(cls, can_id, cmd, value=None):
        """
        Pack control command packet
        :param can_id: CAN ID (int)
        :param cmd: Command code (CMD_xxx)
        :param value: Numerical value (int), if None only the command code is sent
        """
        payload = bytearray([cmd])
        if value is not None:
            # Convert value to 4-byte Big-endian signed integer
            payload.extend(struct.pack('>i', int(value)))

        frame = bytearray()
        frame.append(cls.HEADER)
        frame.append(cls.INFO_BASE | len(payload))
        frame.append(can_id & 0xFF)         # LSB
        frame.append((can_id >> 8) & 0xFF)  # MSB
        frame.extend(payload)
        frame.append(cls.FOOTER)
        return frame

    @classmethod
    def pack_query_frame(cls, can_id, query_type):
        """
        Pack inquiry command packet
        :param can_id: CAN ID (int)
        :param query_type: Query item (QUERY_xxx)
        """
        # Inquiry command is fixed as 0x0F followed by the query sub-code
        payload = bytearray([cls.CMD_INQUIRY, query_type])
        
        frame = bytearray()
        frame.append(cls.HEADER)
        frame.append(cls.INFO_BASE | len(payload))
        frame.append(can_id & 0xFF)
        frame.append((can_id >> 8) & 0xFF)
        frame.extend(payload)
        frame.append(cls.FOOTER)
        return frame

    @staticmethod
    def parse_value(data_bytes):
        """
        Parse the returned data payload
        Typically the return format is [CMD, QUERY_SUB, V0, V1, V2, V3]
        Parses the last 4 bytes.
        """
        if len(data_bytes) < 6:
            return None
        # Parse 4 bytes starting from index 2
        val = struct.unpack('>i', bytes(data_bytes[2:6]))[0]
        return val
    
# ==========================================
# 3. Level Control Parameters
# ==========================================
class DRIVE_MODES:
    # P 模式：加速度(Accel), 減速度(Decel), 目標速度(Target erpm)
    # P 為了不超速，仍保有減速度來進行速限控制
    P1 = {"accel": 200,  "decel": 100,  "speed": 1000}
    P2 = {"accel": 250,  "decel": 100,  "speed": 2000}
    P3 = {"accel": 300, "decel": 100, "speed": 3000}
    P4 = {"accel": 400, "decel": 100, "speed": 4000}
    P5 = {"accel": 500, "decel": 100, "speed": 5000}

class BRAKE_MODES:
    # B 模式：減速度(Decel), 煞車電流(Brake Current)
    # 規定：B 模式加速度(Accel) 永遠為 0
    B1 = {"decel": 200}
    B2 = {"decel": 500}
    B3 = {"decel": 1000}
    B4 = {"decel": 1500}

class NEUTRAL_MODE:
    # N 模式：墮行 (Coasting)
    ACCEL = 0
    DECEL = 100  # 極低的減速，靠自然摩擦力停止
    SPEED = 0

class EMERGENCY_MODE:
    # EB 模式
    DECEL = 100000
    CURRENT = 10000
    SPEED = 0

# ==========================================
# 4. Safety Monitor Thresholds
# ==========================================
class SAFETY_THRESHOLD:
    MAX_TEMP_WARNING = 60.0    # Celsius; warning triggered if exceeded
    MAX_TEMP_CRITICAL = 75.0   # Celsius; emergency brake triggered if exceeded
    
    MIN_VOLTAGE = 20.0         # Volts; undervoltage protection
    MAX_VOLTAGE = 52.0         # Volts; overvoltage protection
    
    MAX_CURRENT_WARNING = 1500 # Units of 10mA (15A); warning for sustained high load
    MAX_CURRENT_CRITICAL = 2500 # Units of 10mA (25A); immediate stop for overload

# Check configuration consistency
if len(MOTOR_LIMITS.DEFAULT_SPEED) != len(CAN_CONFIG.WHEEL_CAN_IDS):
    print("[Warning] Motor IDs and Default Speed list length mismatch!")