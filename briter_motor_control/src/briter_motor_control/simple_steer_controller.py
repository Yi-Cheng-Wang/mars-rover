import time
import threading
import keyboard
from can_bus_manager import CANBusManager
from can_writer import MotorController
from can_reader import MotorReader
from params import CAN_CONFIG, BriterMotorProtocol as Protocol

# ==========================================
# 定義速度段位 (ERPM)
# ==========================================
SPEED_STEPS = [-2000, -1500, -1000, -500, 0, 500, 1000, 1500, 2000]
ZERO_INDEX = 4  # 陣列中 0 的位置

# 預設加減速度
DEFAULT_ACCEL = 500
DEFAULT_DECEL = 1000

def display_dashboard(steps, tgt_idx, act_idx, target_wheel, r_speed):
    """沿用原版的儀表板風格，適配新數值陣列"""
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
            
    print(f"\rControl Wheel:{target_wheel} | Gear:{dash}| Speed(Real): {r_speed:4.0f} \033[K", end="")

# ==========================================
# 簡化版傳動控制器 (保留原版執行緒架構)
# ==========================================
class SimpleThreadedController:
    def __init__(self, ui_wheel_idx=0):
        self.ui_wheel_idx = ui_wheel_idx
        
        self.bus = CANBusManager()
        self.writer = MotorController(self.bus)
        self.reader = MotorReader(self.bus)
        
        # 狀態字典 (支援 4 輪獨立狀態)
        self.raw_target_idx = {i: ZERO_INDEX for i in range(4)}
        self.active_gear_idx = {i: ZERO_INDEX for i in range(4)}
        
        # 保留原版的重傳與同步機制
        self.retransmit_queue = {i: 0 for i in range(4)}
        self.sync_timer = 0
        
        self.running = True
        self.smart_thread = threading.Thread(target=self._smart_transmission_loop, daemon=True)
        self.smart_thread.start()
        print("\n[System] Threaded simple transmission started. [CAN Write Loop & Sync] enabled.")

    def set_target_gear(self, index: int, step_idx: int):
        """設定目標速度索引"""
        if 0 <= step_idx < len(SPEED_STEPS):
            self.raw_target_idx[index] = step_idx

    def _apply_gear(self, index: int, target_erpm: int):
        """實體層 CAN 寫入指令"""
        can_id = CAN_CONFIG.WHEEL_CAN_IDS[index]
        self.bus.send_frame(Protocol.pack_command_frame(can_id, Protocol.CMD_SET_ACCEL_SPEED, DEFAULT_ACCEL))
        self.bus.send_frame(Protocol.pack_command_frame(can_id, Protocol.CMD_SET_DECEL_SPEED, DEFAULT_DECEL))
        self.bus.send_frame(Protocol.pack_command_frame(can_id, Protocol.CMD_SET_SPEED, target_erpm))

    def _smart_transmission_loop(self):
        """完全引用原版的 CAN 寫入執行緒邏輯"""
        while self.running:
            # 1. 同步計時器 (每 15 個 tick 強制發送一次訊號，確保馬達不斷線)
            self.sync_timer += 1
            force_sync = False
            if self.sync_timer >= 15:
                force_sync = True
                self.sync_timer = 0
            
            for w in range(4):
                tgt_idx = self.raw_target_idx[w]
                act_idx_before = self.active_gear_idx[w]
                
                gear_changed = False
                
                # 2. 狀態切換判定
                if act_idx_before != tgt_idx:
                    self.active_gear_idx[w] = tgt_idx
                    gear_changed = True
                
                # 3. 觸發變檔重傳機制 (確保指令確實抵達)
                if gear_changed:
                    self.retransmit_queue[w] = 3
                    
                # 4. 實體層寫入 (Physical layer write)
                if self.retransmit_queue[w] > 0:
                    self._apply_gear(w, SPEED_STEPS[self.active_gear_idx[w]])
                    self.retransmit_queue[w] -= 1
                elif force_sync:
                    self._apply_gear(w, SPEED_STEPS[self.active_gear_idx[w]])

            # 更新 UI (讀取真實馬達速度)
            ui_w = self.ui_wheel_idx
            real_speed = self.reader.cur_speeds[ui_w]
            
            display_dashboard(
                SPEED_STEPS, 
                self.raw_target_idx[ui_w],
                self.active_gear_idx[ui_w],
                ui_w, 
                real_speed
            )
            
            # 原版迴圈延遲
            time.sleep(0.02)

    def stop_all(self):
        self.running = False
        self.writer.close()
        self.reader.close()
        self.bus.close()

# ==========================================
# 互動測試模式 (完全引用原版鍵盤邏輯)
# ==========================================
if __name__ == "__main__":
    TARGET_WHEEL = 0
    steer = SimpleThreadedController(ui_wheel_idx=TARGET_WHEEL)
    
    current_step_idx = ZERO_INDEX
    steer.set_target_gear(TARGET_WHEEL, current_step_idx)
    
    print("\n=======================================================")
    print("  [↑] Upshift (Accelerate)")
    print("  [↓] Downshift (Brake/Reverse)")
    print("  [Space] Instant Stop (0 ERPM)")
    print("  [q] / [Esc] Emergency stop and exit")
    print("=======================================================\n")
    
    # 沿用原版的按鍵計數器
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
                # 若只需控制單輪，保持 TARGET_WHEEL。若要四輪連動，可在此處改成迴圈 set_target_gear
                steer.set_target_gear(TARGET_WHEEL, current_step_idx)
            
            # 原版鍵盤掃描延遲
            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\n\n[System] User forced interrupt test")
    finally:
        # 強制歸零再退出
        for i in range(4):
            steer.set_target_gear(i, ZERO_INDEX)
        time.sleep(0.5) 
        steer.stop_all()
        print("\nSystem closed safely.")