import time
import threading
import os
import csv
import datetime
from params import CAN_CONFIG, BriterMotorProtocol as Protocol

class MotorReader:
    def __init__(self, bus_manager):
        # Receive the central manager passed from outside
        self.bus = bus_manager
        
        self.is_running = True
        self.start_time = time.time()
        
        self.cur_speeds = [0] * len(CAN_CONFIG.WHEEL_CAN_IDS)
        self.cur_positions = [0] * len(CAN_CONFIG.WHEEL_CAN_IDS)

        # Create a directory
        self.log_folder = "./motor_logs"
        if not os.path.exists(self.log_folder):
            os.makedirs(self.log_folder)

        # Register own Callback function with the Manager
        self.bus.register_callback(self._handle_payload)
        
        # Start background polling thread
        self.poll_thread = threading.Thread(target=self._poll_loop, daemon=True)
        self.poll_thread.start()
        print("[Reader] Read logger started, writing data in the background.")

    def _poll_loop(self):
        """Send query commands to all motors periodically in the background"""
        while self.is_running:
            for motor_id in CAN_CONFIG.WHEEL_CAN_IDS:
                self.bus.send_frame(Protocol.pack_query_frame(motor_id, Protocol.QUERY_SPEED))
                time.sleep(0.01)
                self.bus.send_frame(Protocol.pack_query_frame(motor_id, Protocol.QUERY_POS_ABS))
                time.sleep(0.01)
            # Control query frequency (e.g., poll once every 0.5 seconds)
            time.sleep(0.5)

    def _handle_payload(self, can_id, byte_val):
        """Automatically triggered when the Manager receives data"""
        if len(byte_val) < 2: return

        cmd_type = byte_val[0]
        sub_type = byte_val[1]
        
        try:
            index = CAN_CONFIG.WHEEL_CAN_IDS.index(can_id)
        except ValueError:
            return

        timestamp = time.time() - self.start_time

        # Process data returned by the query (0x0F)
        if cmd_type == Protocol.CMD_INQUIRY:
            val = Protocol.parse_value(byte_val)
            if val is None: return
            
            if sub_type == Protocol.QUERY_SPEED:
                self.cur_speeds[index] = val
                self._save_to_csv("speed", can_id, timestamp, val)
                
            elif sub_type == Protocol.QUERY_POS_ABS:
                self.cur_positions[index] = val
                self._save_to_csv("position", can_id, timestamp, val)

    def _save_to_csv(self, data_type, can_id, timestamp, value):
        date_str = datetime.datetime.now().strftime('%Y%m%d')
        file_path = f"{self.log_folder}/{data_type}_id{can_id}_{date_str}.csv"
        
        file_exists = os.path.isfile(file_path)
        with open(file_path, mode='a', newline='') as f:
            writer = csv.writer(f)
            if not file_exists:
                writer.writerow(['Timestamp', 'Value'])
            writer.writerow([f"{timestamp:.3f}", value])

    def close(self):
        self.is_running = False
        if self.poll_thread.is_alive():
            self.poll_thread.join(timeout=1.0)
        print("[Reader] Logger has been closed.")