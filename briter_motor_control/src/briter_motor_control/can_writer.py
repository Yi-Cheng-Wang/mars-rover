import serial
import time
import threading
from params import CAN_CONFIG, MOTOR_LIMITS, BriterMotorProtocol as Protocol

class MotorController:
    def __init__(self):
        try:
            self.ser = serial.Serial(
                port=CAN_CONFIG.PORT,
                baudrate=CAN_CONFIG.BAUDRATE,
                timeout=CAN_CONFIG.TIMEOUT
            )
            print(f"Successfully connected to {CAN_CONFIG.PORT}")
        except Exception as e:
            print(f"Failed to connect to Serial: {e}")
            exit(1)

        self.init_adapter()
        self.is_running = True
        self.heartbeat_thread = threading.Thread(target=self._heartbeat_loop, daemon=True)
        self.heartbeat_thread.start()

    def init_adapter(self):
        """Initializes the USB-to-CAN adapter."""
        self.ser.write(Protocol.get_init_frame())
        print("CAN Adapter initialized.")

    def _heartbeat_loop(self):
        """Maintains communication with the motor via a heartbeat signal."""
        while self.is_running:
            # Sending heartbeat to the first motor as a keep-alive signal
            can_id = CAN_CONFIG.WHEEL_CAN_IDS[0]
            frame = Protocol.pack_command_frame(can_id, Protocol.CMD_HEARTBEAT, 0)
            self.ser.write(frame)
            time.sleep(0.1)

    def set_limits(self, accel: int, decel: int):
        """
        Sets acceleration and deceleration for all wheels.
        :param accel: Acceleration value (erpm/s²)
        :param decel: Deceleration value (erpm/s²)
        """
        for can_id in CAN_CONFIG.WHEEL_CAN_IDS:
            # Send Acceleration
            acc_frame = Protocol.pack_command_frame(can_id, Protocol.CMD_SET_ACCEL_SPEED, accel)
            self.ser.write(acc_frame)
            # Send Deceleration
            dec_frame = Protocol.pack_command_frame(can_id, Protocol.CMD_SET_DECEL_SPEED, decel)
            self.ser.write(dec_frame)
        print(f"Limits set - Accel: {accel}, Decel: {decel}")

    def move_at_speed(self, speeds: list):
        """
        Sets target speeds; motors will accelerate based on previously set limits.
        :param speeds: List of speeds [v1, v2, v3, v4] in erpm
        """
        for i, val in enumerate(speeds):
            if i < len(CAN_CONFIG.WHEEL_CAN_IDS):
                can_id = CAN_CONFIG.WHEEL_CAN_IDS[i]
                frame = Protocol.pack_command_frame(can_id, Protocol.CMD_SET_SPEED, val)
                self.ser.write(frame)
        print(f"Target speeds sent: {speeds}")

    def emergency_brake(self):
        """
        Emergency Brake Procedure:
        1. Set target speed to 0.
        2. Set deceleration to maximum (ensure immediate ramp down).
        3. Apply braking current for a hard lock.
        """
        print("EMERGENCY BRAKE TRIGGERED")
        for can_id in CAN_CONFIG.WHEEL_CAN_IDS:
            # Force speed loop deceleration to a massive value (e.g., 100,000)
            fast_dec_frame = Protocol.pack_command_frame(can_id, Protocol.CMD_SET_DECEL_SPEED, 100000)
            self.ser.write(fast_dec_frame)
            
            # Set speed target to 0
            stop_frame = Protocol.pack_command_frame(can_id, Protocol.CMD_SET_SPEED, 0)
            self.ser.write(stop_frame)
            
            # Apply braking current (e.g., 500 represents 5A, adjust per motor specs)
            brake_cur_frame = Protocol.pack_command_frame(can_id, Protocol.CMD_SET_BRAKE_CUR, 500)
            self.ser.write(brake_cur_frame)

    def close(self):
        """Stops the controller and closes the serial port."""
        self.is_running = False
        self.emergency_brake()
        self.ser.close()
        print("Controller closed.")

# ==========================================
# Control Example
# ==========================================
if __name__ == "__main__":
    controller = MotorController()

    try:
        # 1. Initialize control parameters: Accel 1000, Decel 1000
        # This determines the "smoothness" of the wheel rotation
        controller.set_limits(accel=1000, decel=1000)

        # 2. Gradually accelerate to 2000 erpm
        print("Accelerating to 2000 erpm...")
        controller.move_at_speed([2000, 2000, 2000, 2000])
        time.sleep(5)

        # 3. Test Emergency Brake
        controller.emergency_brake()
        print("Vehicle stopped.")
        time.sleep(2)

    except KeyboardInterrupt:
        controller.emergency_brake()
    finally:
        controller.close()