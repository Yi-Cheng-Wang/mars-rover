import time
import threading
from params import CAN_CONFIG, BriterMotorProtocol as Protocol

class MotorController:
    def __init__(self, bus_manager):
        # Receive the central manager passed from outside
        self.bus = bus_manager
        
        self.is_running = True
        self.heartbeat_thread = threading.Thread(target=self._heartbeat_loop, daemon=True)
        self.heartbeat_thread.start()
        print("[Writer] Motor write controller started, heartbeat thread is running.")

    def _heartbeat_loop(self):
        while self.is_running:
            for can_id in CAN_CONFIG.WHEEL_CAN_IDS:
                frame = Protocol.pack_command_frame(can_id, Protocol.CMD_HEARTBEAT)
                self.bus.send_frame(frame) # Send via Manager
            time.sleep(0.2) 

    def set_limits(self, accel: int, decel: int):
        for can_id in CAN_CONFIG.WHEEL_CAN_IDS:
            acc_frame = Protocol.pack_command_frame(can_id, Protocol.CMD_SET_ACCEL_SPEED, accel)
            self.bus.send_frame(acc_frame)
            
            dec_frame = Protocol.pack_command_frame(can_id, Protocol.CMD_SET_DECEL_SPEED, decel)
            self.bus.send_frame(dec_frame)
        print(f"[Writer] Limits set - Accel: {accel}, Decel: {decel}")

    def move_at_speed(self, speeds: list):
        for i, val in enumerate(speeds):
            if i < len(CAN_CONFIG.WHEEL_CAN_IDS):
                can_id = CAN_CONFIG.WHEEL_CAN_IDS[i]
                frame = Protocol.pack_command_frame(can_id, Protocol.CMD_SET_SPEED, val)
                self.bus.send_frame(frame)

    def emergency_brake(self):
        print("\n[Writer] *** Trigger emergency brake ***")
        for can_id in CAN_CONFIG.WHEEL_CAN_IDS:
            self.bus.send_frame(Protocol.pack_command_frame(can_id, Protocol.CMD_SET_DECEL_SPEED, 100000))
            self.bus.send_frame(Protocol.pack_command_frame(can_id, Protocol.CMD_SET_SPEED, 0))
            self.bus.send_frame(Protocol.pack_command_frame(can_id, Protocol.CMD_SET_BRAKE_CUR, 500))

    def close(self):
        self.is_running = False
        if self.heartbeat_thread.is_alive():
            self.heartbeat_thread.join()
        self.emergency_brake()
        print("[Writer] Controller has been closed.")