import time
import threading
import keyboard
from can_bus_manager import CANBusManager
from can_writer import MotorController
from can_reader import MotorReader
from params import CAN_CONFIG, DRIVE_MODES, BRAKE_MODES, NEUTRAL_MODE, EMERGENCY_MODE, BriterMotorProtocol as Protocol

# ==========================================
# Automatic gear detection
# ==========================================
def get_available_gears():
    """Automatically extract gears from params.py, composed as: EB -> B3 -> B2 -> B1 -> N -> P1 -> P2 -> P3 -> ..."""
    p_gears = sorted([k for k in dir(DRIVE_MODES) if not k.startswith('_')])
    b_gears = sorted([k for k in dir(BRAKE_MODES) if not k.startswith('_')], reverse=True)
    return ['EB'] + b_gears + ['N'] + p_gears

ALL_GEARS = get_available_gears()

def display_dashboard(gears, tgt_idx, act_idx, target_wheel, v_speed, r_speed, cur_accel, warning=""):
    """Terminal dashboard UI"""
    dash = ""
    for i, gear in enumerate(gears):
        if i == tgt_idx and i == act_idx:
            dash += f" [{gear}] "
        elif i == tgt_idx:
            dash += f" >{gear}< "
        elif i == act_idx:
            dash += f" ({gear}) "
        else:
            dash += f"  {gear}  "
            
    print(f"\rControl Wheel:{target_wheel} | Gear:{dash}| Speed(V/R): {v_speed:4.0f}/{r_speed:4.0f} | Accel: {cur_accel:3.0f}{warning}\033[K", end="")

# ==========================================
# Smart Transmission Controller
# ==========================================
class SmartSteerController:
    def __init__(self, ui_wheel_idx=0):
        self.ui_wheel_idx = ui_wheel_idx
        
        self.bus = CANBusManager()
        self.writer = MotorController(self.bus)
        self.reader = MotorReader(self.bus)
        
        self.raw_target_idx = {i: ALL_GEARS.index('N') for i in range(4)}
        self.last_raw_idx = {i: ALL_GEARS.index('N') for i in range(4)}
        self.stable_target_idx = {i: ALL_GEARS.index('N') for i in range(4)}
        self.debounce_ticks = {i: 0 for i in range(4)}
        
        self.active_gear_idx = {i: ALL_GEARS.index('N') for i in range(4)}
        self.virtual_speed = {i: 0.0 for i in range(4)} 
        
        self.current_accel = {i: 0.0 for i in range(4)}
        self.last_accel_update = {i: time.time() for i in range(4)}
        
        self.retransmit_queue = {i: 0 for i in range(4)}
        self.sync_timer = 0
        self.warning_until = 0.0
        
        self.last_update_time = time.time()
        self.running = True
        self.smart_thread = threading.Thread(target=self._smart_transmission_loop, daemon=True)
        self.smart_thread.start()
        print("\n[System] Smart transmission system started, [Continuous smooth acceleration & Closed-loop fail-safe protection] enabled.")

    def set_target_gear(self, index: int, gear_name: str):
        if gear_name in ALL_GEARS:
            self.raw_target_idx[index] = ALL_GEARS.index(gear_name)

    def _apply_gear(self, index: int, gear_name: str):
        can_id = CAN_CONFIG.WHEEL_CAN_IDS[index]
        
        if gear_name == 'EB':
            self.bus.send_frame(Protocol.pack_command_frame(can_id, Protocol.CMD_SET_ACCEL_SPEED, 0))
            self.bus.send_frame(Protocol.pack_command_frame(can_id, Protocol.CMD_SET_DECEL_SPEED, EMERGENCY_MODE.DECEL))
            self.bus.send_frame(Protocol.pack_command_frame(can_id, Protocol.CMD_SET_SPEED, 0))
            self.bus.send_frame(Protocol.pack_command_frame(can_id, Protocol.CMD_SET_BRAKE_CUR, EMERGENCY_MODE.CURRENT))

        elif gear_name == 'N':
            self.bus.send_frame(Protocol.pack_command_frame(can_id, Protocol.CMD_SET_ACCEL_SPEED, NEUTRAL_MODE.ACCEL))
            self.bus.send_frame(Protocol.pack_command_frame(can_id, Protocol.CMD_SET_DECEL_SPEED, NEUTRAL_MODE.DECEL))
            self.bus.send_frame(Protocol.pack_command_frame(can_id, Protocol.CMD_SET_SPEED, NEUTRAL_MODE.SPEED))
        
        elif gear_name.startswith('B'):
            config = getattr(BRAKE_MODES, gear_name)
            self.bus.send_frame(Protocol.pack_command_frame(can_id, Protocol.CMD_SET_ACCEL_SPEED, 0))
            self.bus.send_frame(Protocol.pack_command_frame(can_id, Protocol.CMD_SET_DECEL_SPEED, config["decel"]))
            self.bus.send_frame(Protocol.pack_command_frame(can_id, Protocol.CMD_SET_SPEED, 0))
            if "current" in config:
                self.bus.send_frame(Protocol.pack_command_frame(can_id, Protocol.CMD_SET_BRAKE_CUR, config["current"]))
            
        elif gear_name.startswith('P'):
            config = getattr(DRIVE_MODES, gear_name)
            self.bus.send_frame(Protocol.pack_command_frame(can_id, Protocol.CMD_SET_ACCEL_SPEED, int(self.current_accel[index])))
            self.bus.send_frame(Protocol.pack_command_frame(can_id, Protocol.CMD_SET_DECEL_SPEED, config["decel"]))
            self.bus.send_frame(Protocol.pack_command_frame(can_id, Protocol.CMD_SET_SPEED, config["speed"]))

    def _get_gear_speed(self, gear_name):
        if gear_name in ['N', 'EB'] or gear_name.startswith('B'): return 0.0
        return getattr(DRIVE_MODES, gear_name)["speed"]

    def _get_gear_accel(self, gear_name):
        if gear_name in ['N', 'EB'] or gear_name.startswith('B'): return 0.0
        return getattr(DRIVE_MODES, gear_name)["accel"]

    def _get_gear_decel(self, gear_name):
        if gear_name == 'N': return NEUTRAL_MODE.DECEL
        if gear_name == 'EB': return EMERGENCY_MODE.DECEL
        if gear_name.startswith('B'): return getattr(BRAKE_MODES, gear_name)["decel"]
        return getattr(DRIVE_MODES, gear_name)["decel"]

    def _smart_transmission_loop(self):
        while self.running:
            current_time = time.time()
            dt = current_time - self.last_update_time
            self.last_update_time = current_time
            
            self.sync_timer += 1
            force_sync = False
            if self.sync_timer >= 15:
                force_sync = True
                self.sync_timer = 0
            
            for w in range(4):
                real_speed = self.reader.cur_speeds[w]
                
                # Stall/surge protection: error greater than 1000 triggers EB
                if abs(self.virtual_speed[w] - real_speed) > 1000:
                    self.raw_target_idx[w] = ALL_GEARS.index('EB')
                    self.stable_target_idx[w] = ALL_GEARS.index('EB')
                    self.warning_until = current_time + 2.0  
                
                raw_idx = self.raw_target_idx[w]
                raw_name = ALL_GEARS[raw_idx]
                
                if raw_idx != self.last_raw_idx[w]:
                    self.debounce_ticks[w] = 0
                    self.last_raw_idx[w] = raw_idx
                else:
                    self.debounce_ticks[w] += 1
                
                if raw_name.startswith('B') or raw_name in ['N', 'EB']:
                    self.stable_target_idx[w] = raw_idx
                elif self.debounce_ticks[w] >= 2:
                    self.stable_target_idx[w] = raw_idx
                
                tgt_idx = self.stable_target_idx[w]
                
                # [Record state before shifting] Used to determine whether it is "cruising" or "continuous acceleration"
                act_idx_before = self.active_gear_idx[w]
                act_name_before = ALL_GEARS[act_idx_before]
                act_speed_before = self._get_gear_speed(act_name_before)
                # If virtual speed reaches 99% of target speed, determine as cruising stable state
                is_cruising = (self.virtual_speed[w] >= act_speed_before * 0.99)
                
                gear_changed = False
                
                # Rule 1: Target is brake or N
                if ALL_GEARS[tgt_idx].startswith('B') or ALL_GEARS[tgt_idx] in ['N', 'EB']:
                    if act_idx_before != tgt_idx:
                        self.active_gear_idx[w] = tgt_idx
                        gear_changed = True
                        self.virtual_speed[w] = real_speed 

                # Rule 2: Target is drive gear (P)
                elif ALL_GEARS[tgt_idx].startswith('P'):
                    if act_name_before.startswith('B') or act_name_before in ['N', 'EB']:
                        self.active_gear_idx[w] = ALL_GEARS.index('P1')
                        gear_changed = True
                        self.virtual_speed[w] = real_speed
                    else:
                        act_speed = self._get_gear_speed(act_name_before)
                        if tgt_idx > act_idx_before and self.virtual_speed[w] >= act_speed * 0.95:
                            self.active_gear_idx[w] = act_idx_before + 1
                            gear_changed = True
                        elif tgt_idx < act_idx_before and self.virtual_speed[w] <= act_speed * 1.05:
                            self.active_gear_idx[w] = act_idx_before - 1
                            gear_changed = True

                # Context-aware anti-surge
                if gear_changed:
                    self.retransmit_queue[w] = 3
                    
                    new_act_name = ALL_GEARS[self.active_gear_idx[w]]
                    if new_act_name.startswith('P'):
                        target_accel = self._get_gear_accel(new_act_name)
                        is_upshift = self.active_gear_idx[w] > act_idx_before
                        
                        # Case A: Start from standstill or brake -> Reset to 200 for safe start
                        if act_name_before in ['N', 'EB'] or act_name_before.startswith('B'):
                            self.current_accel[w] = min(200.0, target_accel)
                            
                        # Case B: Upshift after cruising stable (e.g., stable at P4 suddenly switch to P5) -> Reset to 200 to avoid thrust surge
                        elif is_upshift and is_cruising:
                            self.current_accel[w] = min(200.0, target_accel)
                            
                        # Case C: Upshift during continuous acceleration (virtual speed has not reached upper limit) -> Do not reset! Maintain strong thrust for seamless transition
                        else:
                            pass 
                    else:
                        self.current_accel[w] = 0.0
                
                # Physical layer write
                if self.retransmit_queue[w] > 0:
                    self._apply_gear(w, ALL_GEARS[self.active_gear_idx[w]])
                    self.retransmit_queue[w] -= 1
                elif force_sync:
                    self._apply_gear(w, ALL_GEARS[self.active_gear_idx[w]])
                    
                # ========================================================
                # Dynamic acceleration incrementer (Jerk Limiter)
                # ========================================================
                target_accel = self._get_gear_accel(ALL_GEARS[self.active_gear_idx[w]])
                
                if target_accel > self.current_accel[w]:
                    if current_time - self.last_accel_update[w] >= 0.2: 
                        self.current_accel[w] += 100.0
                        if self.current_accel[w] > target_accel:
                            self.current_accel[w] = target_accel
                            
                        self.last_accel_update[w] = current_time
                        
                        if ALL_GEARS[self.active_gear_idx[w]].startswith('P'):
                            self.bus.send_frame(Protocol.pack_command_frame(CAN_CONFIG.WHEEL_CAN_IDS[w], Protocol.CMD_SET_ACCEL_SPEED, int(self.current_accel[w])))
                            
                elif target_accel < self.current_accel[w]:
                    self.current_accel[w] = target_accel
                    self.last_accel_update[w] = current_time
                                
                # Virtual speed simulation update
                act_speed = self._get_gear_speed(ALL_GEARS[self.active_gear_idx[w]])
                act_decel = self._get_gear_decel(ALL_GEARS[self.active_gear_idx[w]])
                
                if self.virtual_speed[w] < act_speed:
                    self.virtual_speed[w] += self.current_accel[w] * dt
                    self.virtual_speed[w] = min(self.virtual_speed[w], act_speed) 
                elif self.virtual_speed[w] > act_speed:
                    self.virtual_speed[w] -= act_decel * dt
                    self.virtual_speed[w] = max(self.virtual_speed[w], act_speed)

            # Update UI
            ui_w = self.ui_wheel_idx
            real_speed = self.reader.cur_speeds[ui_w]
            warning_str = "    [Gap too large, EB intervention]" if current_time < self.warning_until else ""
            
            display_dashboard(
                ALL_GEARS, 
                self.raw_target_idx[ui_w],
                self.active_gear_idx[ui_w],
                ui_w, 
                self.virtual_speed[ui_w],
                real_speed,
                self.current_accel[ui_w],
                warning_str
            )
            
            time.sleep(0.02)

    def apply_emergency_brake(self):
        self.writer.emergency_brake()
        for i in range(4):
            self.raw_target_idx[i] = ALL_GEARS.index('EB')
            self.stable_target_idx[i] = ALL_GEARS.index('EB')
            self.active_gear_idx[i] = ALL_GEARS.index('EB')
            self.retransmit_queue[i] = 3  
            self.virtual_speed[i] = 0.0
            self.current_accel[i] = 0.0

    def stop_all(self):
        self.running = False
        self.writer.close()
        self.reader.close()
        self.bus.close()

# ==========================================
# Interactive test mode
# ==========================================
if __name__ == "__main__":
    TARGET_WHEEL = 0
    steer = SmartSteerController(ui_wheel_idx=TARGET_WHEEL)
    
    current_gear_idx = ALL_GEARS.index('B3')
    steer.set_target_gear(TARGET_WHEEL, 'B3')
    
    print("\n=======================================================")
    print("  [↑] Upshift (Accelerate)")
    print("  [↓] Downshift (Brake)")
    print("  [Space] Instant return to N gear coasting")
    print("  [q] / [Esc] Emergency stop and exit")
    print("=======================================================\n")
    
    key_counters = {'up': 0, 'down': 0}
    last_space = False
    
    try:
        while True:
            changed = False
            
            if keyboard.is_pressed('q') or keyboard.is_pressed('esc'):
                print("\n\n[System] Exit command received, executing emergency stop...")
                break
                
            space_pressed = keyboard.is_pressed('space')
            if space_pressed and not last_space:
                current_gear_idx = ALL_GEARS.index('N')
                changed = True
            last_space = space_pressed
            
            if keyboard.is_pressed('up'):
                key_counters['up'] += 1
                c = key_counters['up']
                if c == 1 or (c > 8 and (c - 8) % 2 == 0):
                    if current_gear_idx < len(ALL_GEARS) - 1:
                        current_gear_idx += 1
                        changed = True
            else:
                key_counters['up'] = 0
                
            if keyboard.is_pressed('down'):
                key_counters['down'] += 1
                c = key_counters['down']
                if c == 1 or (c > 8 and (c - 8) % 2 == 0):
                    if current_gear_idx > 0:
                        current_gear_idx -= 1
                        changed = True
            else:
                key_counters['down'] = 0
                
            if changed:
                steer.set_target_gear(TARGET_WHEEL, ALL_GEARS[current_gear_idx])
            
            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\n\n[System] User forced interrupt test")
    finally:
        steer.apply_emergency_brake()
        time.sleep(0.5) 
        steer.stop_all()
        print("\nSystem closed safely.")