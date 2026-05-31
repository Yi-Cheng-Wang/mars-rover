import time
import threading
import keyboard
from can_bus_manager import CANBusManager
from can_writer import MotorController
from can_reader import MotorReader
from params import CAN_CONFIG, BriterMotorProtocol as Protocol

# ==========================================
# Define speed steps (ERPM)
# ==========================================
SPEED_STEPS = [-2000, -1500, -1000, -500, 0, 500, 1000, 1500, 2000]
ZERO_INDEX = 4  # Index corresponding to 0 ERPM

# Default acceleration and deceleration
DEFAULT_ACCEL = 500
DEFAULT_DECEL = 1000

def display_dashboard(steps, tgt_idx, act_idx, ui_wheel, r_speed):
    """Retain the original dashboard style, adapted for the new numerical array"""
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
            
    print(f"\rUI Wheel:{ui_wheel} | Gear:{dash}| Speed(Real): {r_speed:4.0f} \033[K", end="")

# ==========================================
# Simplified Transmission Controller
# ==========================================
class SimpleThreadedController:
    def __init__(self, ui_wheel_idx=0):
        self.ui_wheel_idx = ui_wheel_idx
        
        self.bus = CANBusManager()
        self.writer = MotorController(self.bus)
        self.reader = MotorReader(self.bus)
        
        # State dictionaries (Supports 6 independent wheels)
        self.raw_target_idx = {i: ZERO_INDEX for i in range(6)}
        self.active_gear_idx = {i: ZERO_INDEX for i in range(6)}
        
        self.retransmit_queue = {i: 0 for i in range(6)}
        self.sync_timer = 0
        
        self.running = True
        self.smart_thread = threading.Thread(target=self._smart_transmission_loop, daemon=True)
        self.smart_thread.start()
        print("\n[System] Threaded simple transmission started. [CAN Write Loop & Sync] enabled for 6 wheels.")

    def set_target_gear(self, index: int, step_idx: int):
        """Set target speed index"""
        if 0 <= step_idx < len(SPEED_STEPS):
            self.raw_target_idx[index] = step_idx

    def _apply_gear(self, index: int, target_erpm: int, is_sync: bool = False):
        """
        Physical layer CAN write command.
        [Fix applied]: Only send Accel/Decel parameters when actively shifting gears.
        Do NOT send them during routine synchronization (is_sync=True) to prevent 
        motor controller PID reset/twitching at 0 ERPM.
        """
        # Ensure your CAN_CONFIG.WHEEL_CAN_IDS in params.py has at least 6 items!
        can_id = CAN_CONFIG.WHEEL_CAN_IDS[index]
        
        if not is_sync:
            # Only update acceleration and deceleration when the target actually changes
            self.bus.send_frame(Protocol.pack_command_frame(can_id, Protocol.CMD_SET_ACCEL_SPEED, DEFAULT_ACCEL))
            self.bus.send_frame(Protocol.pack_command_frame(can_id, Protocol.CMD_SET_DECEL_SPEED, DEFAULT_DECEL))
            
        # Always send the target speed (acts as the heartbeat during sync)
        self.bus.send_frame(Protocol.pack_command_frame(can_id, Protocol.CMD_SET_SPEED, target_erpm))

    def _smart_transmission_loop(self):
        """Fully references the original CAN write thread logic"""
        while self.running:
            # 1. Sync timer (Force send signal every 15 ticks)
            self.sync_timer += 1
            force_sync = False
            if self.sync_timer >= 15:
                force_sync = True
                self.sync_timer = 0
            
            # Loop through all 6 wheels
            for w in range(6):
                tgt_idx = self.raw_target_idx[w]
                act_idx_before = self.active_gear_idx[w]
                
                gear_changed = False
                
                # 2. State change detection
                if act_idx_before != tgt_idx:
                    self.active_gear_idx[w] = tgt_idx
                    gear_changed = True
                
                # 3. Trigger gear shift retransmission mechanism
                if gear_changed:
                    self.retransmit_queue[w] = 3
                    
                # 4. Physical layer write
                if self.retransmit_queue[w] > 0:
                    # Not a sync, it's a real gear shift -> send full parameters (is_sync=False)
                    self._apply_gear(w, SPEED_STEPS[self.active_gear_idx[w]], is_sync=False)
                    self.retransmit_queue[w] -= 1
                elif force_sync:
                    # Just a routine sync heartbeat -> only send speed command (is_sync=True)
                    self._apply_gear(w, SPEED_STEPS[self.active_gear_idx[w]], is_sync=True)

            # Update UI
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
    UI_TARGET_WHEEL = 0  # Wheel to monitor on the dashboard
    steer = SimpleThreadedController(ui_wheel_idx=UI_TARGET_WHEEL)
    
    current_step_idx = ZERO_INDEX
    
    # Initialize all 6 wheels to zero
    for i in range(6):
        steer.set_target_gear(i, current_step_idx)
    
    print("\n=======================================================")
    print("  [↑] Upshift (Accelerate)")
    print("  [↓] Downshift (Brake/Reverse)")
    print("  [Space] Instant Stop (0 ERPM)")
    print("  [q] / [Esc] Emergency stop and exit")
    print("=======================================================\n")
    
    key_counters = {'up': 0, 'down': 0}
    last_space = False
    
    try:
        while True:
            changed = False
            
            if keyboard.is_pressed('q') or keyboard.is_pressed('esc'):
                print("\n\n[System] Exit command received...")
                break
                
            space_pressed = keyboard.is_pressed('space')
            if space_pressed and not last_space:
                current_step_idx = ZERO_INDEX
                changed = True
            last_space = space_pressed
            
            if keyboard.is_pressed('up'):
                key_counters['up'] += 1
                c = key_counters['up']
                if c == 1 or (c > 8 and (c - 8) % 2 == 0):
                    if current_step_idx < len(SPEED_STEPS) - 1:
                        current_step_idx += 1
                        changed = True
            else:
                key_counters['up'] = 0
                
            if keyboard.is_pressed('down'):
                key_counters['down'] += 1
                c = key_counters['down']
                if c == 1 or (c > 8 and (c - 8) % 2 == 0):
                    if current_step_idx > 0:
                        current_step_idx -= 1
                        changed = True
            else:
                key_counters['down'] = 0
                
            if changed:
                # Apply the target gear to ALL 6 wheels
                for w in range(6):
                    steer.set_target_gear(w, current_step_idx)
            
            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\n\n[System] User forced interrupt test")
    finally:
        for i in range(6):
            steer.set_target_gear(i, ZERO_INDEX)
        time.sleep(0.5) 
        steer.stop_all()
        print("\nSystem closed safely.")