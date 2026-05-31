import sys
import time
import threading
from can_bus_manager import CANBusManager
from can_writer import MotorController
from can_reader import MotorReader
from params import CAN_CONFIG, BriterMotorProtocol as Protocol

# OS-specific imports for non-blocking keyboard reading
is_windows = sys.platform == 'win32'
if is_windows:
    import msvcrt
else:
    import tty
    import termios
    import select

# ==========================================
# Define speed steps (ERPM)
# ==========================================
SPEED_STEPS = [-2000, -1500, -1000, 0, 1000, 1500, 2000]
ZERO_INDEX = 3  # Index corresponding to 0 ERPM

DEFAULT_ACCEL = 500
DEFAULT_DECEL = 2000

def display_dashboard(steps, tgt_idx, act_idx, ui_wheel, r_speed):
    dash = ""
    for i, speed in enumerate(steps):
        if i == tgt_idx and i == act_idx:
            dash += f" [{speed}] "
        elif i == tgt_idx:
            dash += f" >{speed}< "
        elif i == act_idx:
            dash += f" ({speed}) "
        else:
            dash += f"  {speed}  "
            
    print(f"\rUI Wheel:{ui_wheel} | Gear:{dash}| Speed(Real): {r_speed:4.0f} \033[K", end="", flush=True)

# ==========================================
# Cross-Platform Keyboard Reader
# ==========================================
class CrossPlatformKeyReader:
    def __init__(self):
        self.is_windows = sys.platform == 'win32'
        if not self.is_windows:
            self.fd = sys.stdin.fileno()
            self.old_settings = termios.tcgetattr(self.fd)
            tty.setcbreak(self.fd)

    def get_key(self):
        """Non-blocking key read. Returns 'w', 's', 'space', 'q', 'esc', or None."""
        if self.is_windows:
            if msvcrt.kbhit():
                ch = msvcrt.getch()
                if ch.lower() == b'w': return 'w'
                elif ch.lower() == b's': return 's'
                elif ch == b' ': return 'space'
                elif ch.lower() == b'q': return 'q'
                elif ch == b'\x1b': return 'esc'
                elif ch == b'\x03': raise KeyboardInterrupt  # Ctrl+C
        else:
            if select.select([sys.stdin], [], [], 0.0)[0]:
                ch = sys.stdin.read(1)
                if ch.lower() == 'w': return 'w'
                elif ch.lower() == 's': return 's'
                elif ch == ' ': return 'space'
                elif ch.lower() == 'q': return 'q'
                elif ch == '\x1b': return 'esc'
                elif ch == '\x03': raise KeyboardInterrupt # Ctrl+C
        return None

    def cleanup(self):
        """Restore terminal settings (critical for Linux)"""
        if not self.is_windows:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)

# ==========================================
# Simplified Transmission Controller
# ==========================================
class SimpleThreadedController:
    def __init__(self, ui_wheel_idx=0):
        self.ui_wheel_idx = ui_wheel_idx
        
        self.bus = CANBusManager()
        self.writer = MotorController(self.bus)
        self.reader = MotorReader(self.bus)
        
        self.raw_target_idx = {i: ZERO_INDEX for i in range(6)}
        self.active_gear_idx = {i: ZERO_INDEX for i in range(6)}
        
        self.retransmit_queue = {i: 0 for i in range(6)}
        self.sync_timer = 0
        
        self.running = True
        self.smart_thread = threading.Thread(target=self._smart_transmission_loop, daemon=True)
        self.smart_thread.start()
        print("\n[System] Threaded simple transmission started. [CAN Write Loop & Sync] enabled for 6 wheels.")

    def set_target_gear(self, index: int, step_idx: int):
        if 0 <= step_idx < len(SPEED_STEPS):
            self.raw_target_idx[index] = step_idx

    def _apply_gear(self, index: int, target_erpm: int, is_sync: bool = False):
        can_id = CAN_CONFIG.WHEEL_CAN_IDS[index]
        if not is_sync:
            self.bus.send_frame(Protocol.pack_command_frame(can_id, Protocol.CMD_SET_ACCEL_SPEED, DEFAULT_ACCEL))
            self.bus.send_frame(Protocol.pack_command_frame(can_id, Protocol.CMD_SET_DECEL_SPEED, DEFAULT_DECEL))
            
        self.bus.send_frame(Protocol.pack_command_frame(can_id, Protocol.CMD_SET_SPEED, target_erpm))

    def _smart_transmission_loop(self):
        while self.running:
            self.sync_timer += 1
            force_sync = False
            if self.sync_timer >= 15:
                force_sync = True
                self.sync_timer = 0
            
            for w in range(6):
                tgt_idx = self.raw_target_idx[w]
                act_idx_before = self.active_gear_idx[w]
                gear_changed = False
                
                if act_idx_before != tgt_idx:
                    self.active_gear_idx[w] = tgt_idx
                    gear_changed = True
                
                if gear_changed:
                    self.retransmit_queue[w] = 3
                    
                if self.retransmit_queue[w] > 0:
                    self._apply_gear(w, SPEED_STEPS[self.active_gear_idx[w]], is_sync=False)
                    self.retransmit_queue[w] -= 1
                elif force_sync:
                    self._apply_gear(w, SPEED_STEPS[self.active_gear_idx[w]], is_sync=True)

            ui_w = self.ui_wheel_idx
            real_speed = self.reader.cur_speeds[ui_w]
            
            display_dashboard(
                SPEED_STEPS, 
                self.raw_target_idx[ui_w],
                self.active_gear_idx[ui_w],
                ui_w, 
                real_speed
            )
            time.sleep(0.02)

    def stop_all(self):
        self.running = False
        self.writer.close()
        self.reader.close()
        self.bus.close()

# ==========================================
# Interactive test mode
# ==========================================
if __name__ == "__main__":
    UI_TARGET_WHEEL = 0  
    steer = SimpleThreadedController(ui_wheel_idx=UI_TARGET_WHEEL)
    current_step_idx = ZERO_INDEX
    
    for i in range(6):
        steer.set_target_gear(i, current_step_idx)
    
    print("\n=======================================================")
    print("  [W] Upshift (Accelerate)")
    print("  [S] Downshift (Brake/Reverse)")
    print("  [Space] Instant Stop (0 ERPM)")
    print("  [Q] / [Esc] Emergency stop and exit")
    print("=======================================================\n")
    
    key_reader = CrossPlatformKeyReader()
    
    last_shift_time = 0 
    shift_cooldown = 0.15  # Cooldown to prevent shifting too fast
    
    try:
        while True:
            key = key_reader.get_key()
            changed = False
            current_time = time.time()
            
            if key in ['q', 'esc']:
                print("\n\n[System] Exit command received...")
                break
                
            elif key == 'space':
                current_step_idx = ZERO_INDEX
                changed = True
                
            elif key == 'w':
                if current_time - last_shift_time > shift_cooldown:
                    if current_step_idx < len(SPEED_STEPS) - 1:
                        current_step_idx += 1
                        changed = True
                        last_shift_time = current_time
                        
            elif key == 's':
                if current_time - last_shift_time > shift_cooldown:
                    if current_step_idx > 0:
                        current_step_idx -= 1
                        changed = True
                        last_shift_time = current_time
            
            if changed:
                for w in range(6):
                    steer.set_target_gear(w, current_step_idx)
                    
            time.sleep(0.02)

    except KeyboardInterrupt:
        print("\n\n[System] User forced interrupt test")
    finally:
        key_reader.cleanup()
        
        for i in range(6):
            steer.set_target_gear(i, ZERO_INDEX)
        time.sleep(0.5) 
        steer.stop_all()
        print("\nSystem closed safely.")