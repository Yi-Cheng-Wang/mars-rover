import time
from can_writer import MotorController
from params import CAN_CONFIG, DRIVE_MODES, BRAKE_MODES, NEUTRAL_MODE, EMERGENCY_MODE, BriterMotorProtocol as Protocol

class SteerVelocityController:
    def __init__(self):
        # Inherit and initialize the base motor controller
        self.base = MotorController()
        print("Steer Velocity Controller Module Loaded.")

    def set_wheel_drive(self, index: int, level: str):
        """
        Set a specific wheel to P (Power/Drive) mode.
        :param index: Wheel index (0-3)
        :param level: Drive level string ('P1', 'P2', 'P3')
        """
        config = getattr(DRIVE_MODES, level)
        can_id = CAN_CONFIG.WHEEL_CAN_IDS[index]
        
        # 1. First, set physical limits (Acceleration and Deceleration)
        self.base.ser.write(Protocol.pack_command_frame(can_id, Protocol.CMD_SET_ACCEL_SPEED, config["accel"]))
        self.base.ser.write(Protocol.pack_command_frame(can_id, Protocol.CMD_SET_DECEL_SPEED, config["decel"]))
        
        # 2. Set the target speed
        self.base.ser.write(Protocol.pack_command_frame(can_id, Protocol.CMD_SET_SPEED, config["speed"]))
        print(f"Wheel {index} (ID: {can_id:X}) set to {level}: Speed {config['speed']}")

    def set_wheel_brake(self, index: int, level: str):
        """
        Set a specific wheel to B (Brake) mode.
        :param index: Wheel index (0-3)
        :param level: Brake level string ('B1', 'B2', 'B3')
        """
        config = getattr(BRAKE_MODES, level)
        can_id = CAN_CONFIG.WHEEL_CAN_IDS[index]
        
        # 1. B-Mode Rule: Set acceleration to 0 to ensure no power is generated
        self.base.ser.write(Protocol.pack_command_frame(can_id, Protocol.CMD_SET_ACCEL_SPEED, 0))
        
        # 2. Set deceleration and set target speed to 0
        self.base.ser.write(Protocol.pack_command_frame(can_id, Protocol.CMD_SET_DECEL_SPEED, config["decel"]))
        self.base.ser.write(Protocol.pack_command_frame(can_id, Protocol.CMD_SET_SPEED, 0))
        
        # 3. Apply corresponding braking current level
        self.base.ser.write(Protocol.pack_command_frame(can_id, Protocol.CMD_SET_BRAKE_CUR, config["current"]))
        print(f"Wheel {index} (ID: {can_id:X}) set to {level}: Brake Decel {config['decel']}")

    def set_wheel_neutral(self, index: int):
        """ N Mode: Set wheel to neutral (Coasting). """
        can_id = CAN_CONFIG.WHEEL_CAN_IDS[index]
        self.base.ser.write(Protocol.pack_command_frame(can_id, Protocol.CMD_SET_ACCEL_SPEED, NEUTRAL_MODE.ACCEL))
        self.base.ser.write(Protocol.pack_command_frame(can_id, Protocol.CMD_SET_DECEL_SPEED, NEUTRAL_MODE.DECEL))
        self.base.ser.write(Protocol.pack_command_frame(can_id, Protocol.CMD_SET_SPEED, NEUTRAL_MODE.SPEED))
        print(f"Wheel {index} set to NEUTRAL (Coasting)")

    def apply_emergency_brake(self):
        """ EB Mode: Trigger immediate emergency brake for all wheels. """
        self.base.emergency_brake()

    def stop_all(self):
        """ Close connections. """
        self.base.close()

# ==========================================
# Control Example
# ==========================================
if __name__ == "__main__":
    steer = SteerVelocityController()

    try:
        # Example: Left side wheels (0, 1) at P2, Right side wheels (2, 3) at P1
        steer.set_wheel_drive(0, "P2")
        steer.set_wheel_drive(1, "P2")
        steer.set_wheel_drive(2, "P1")
        steer.set_wheel_drive(3, "P1")
        
        time.sleep(4)

        # Example: Switch to B1 for smooth deceleration
        print("Switching to Service Brake B1...")
        for i in range(4):
            steer.set_wheel_brake(i, "B1")
        
        time.sleep(2)

        # Example: Finally enter Neutral coasting until stationary
        print("Entering Neutral...")
        for i in range(4):
            steer.set_wheel_neutral(i)

    except KeyboardInterrupt:
        steer.apply_emergency_brake()
    finally:
        steer.stop_all()