import serial
import time
import threading
import os
import csv
import datetime
from params import CAN_CONFIG, BriterMotorProtocol as Protocol

class MotorReader:
    def __init__(self, shared_serial=None):
        # 1. Initialize Serial connection (Share existing serial object if provided to avoid port occupancy)
        if shared_serial:
            self.ser = shared_serial
        else:
            try:
                self.ser = serial.Serial(
                    port=CAN_CONFIG.PORT,
                    baudrate=CAN_CONFIG.BAUDRATE,
                    timeout=CAN_CONFIG.TIMEOUT
                )
                print(f"Reader connected to {CAN_CONFIG.PORT}")
            except Exception as e:
                print(f"Failed to connect to Serial: {e}")
                exit(1)

        # 2. State variables
        self.is_running = True
        self.start_time = time.time()
        
        # Data cache
        self.cur_speeds = [0] * len(CAN_CONFIG.WHEEL_CAN_IDS)
        self.cur_positions = [0] * len(CAN_CONFIG.WHEEL_CAN_IDS)

        # File path configuration
        self.log_folder = "./motor_logs"
        if not os.path.exists(self.log_folder):
            os.makedirs(self.log_folder)

        # 3. Start background reading thread
        self.read_thread = threading.Thread(target=self._read_loop, daemon=True)
        self.read_thread.start()

    def _read_loop(self):
        """Background loop to continuously listen to the serial port and parse CAN data"""
        while self.is_running:
            if self.ser.in_waiting > 0:
                self._parse_serial_buffer()
            time.sleep(0.001)  # Reduce CPU load

    def _parse_serial_buffer(self):
        """Parse packets following the AA C0 ... 55 protocol"""
        try:
            # Find Start Byte
            header = self.ser.read(1)
            if header != b'\xAA':
                return

            # Read Info Byte and get DLC (Data Length Code)
            info = self.ser.read(1)
            dlc = info[0] & 0x0F

            # Read CAN ID (2 Bytes, Little-endian)
            id_lsb = self.ser.read(1)[0]
            id_msb = self.ser.read(1)[0]
            can_id = (id_msb << 8) | id_lsb

            # Read Payload
            byte_val = list(self.ser.read(dlc))
            
            # Read End Byte
            footer = self.ser.read(1)
            if footer != b'\x55':
                return

            # Parse data based on command type
            self._handle_payload(can_id, byte_val)

        except Exception as e:
            print(f"Parsing Error: {e}")

    def _handle_payload(self, can_id, byte_val):
        """Identify returned data type and store it"""
        if len(byte_val) < 2:
            return

        cmd_type = byte_val[0]
        sub_type = byte_val[1]
        
        # Get wheel index
        try:
            index = CAN_CONFIG.WHEEL_CAN_IDS.index(can_id)
        except ValueError:
            return

        timestamp = time.time() - self.start_time

        # Handle Inquiry response (0x0F)
        if cmd_type == Protocol.CMD_INQUIRY:
            val = Protocol.parse_value(byte_val)
            
            if sub_type == Protocol.QUERY_SPEED:
                self.cur_speeds[index] = val
                self._save_to_csv("speed", can_id, timestamp, val)
                
            elif sub_type == Protocol.QUERY_POS_ABS:
                self.cur_positions[index] = val
                self._save_to_csv("position", can_id, timestamp, val)

    def _save_to_csv(self, data_type, can_id, timestamp, value):
        """Write data to a CSV file"""
        date_str = datetime.datetime.now().strftime('%Y%m%d')
        file_path = f"{self.log_folder}/{data_type}_id{can_id}_{date_str}.csv"
        
        file_exists = os.path.isfile(file_path)
        with open(file_path, mode='a', newline='') as f:
            writer = csv.writer(f)
            if not file_exists:
                writer.writerow(['Timestamp', 'Value'])
            writer.writerow([f"{timestamp:.3f}", value])

    def send_inquiry(self, can_id, query_type):
        """Send inquiry request to a specific motor"""
        frame = Protocol.pack_query_frame(can_id, query_type)
        self.ser.write(frame)

    def close(self):
        self.is_running = False
        if self.read_thread.is_alive():
            self.read_thread.join(timeout=1.0)
        print("Reader closed.")

# ==========================================
# Standalone Test Example
# ==========================================
if __name__ == "__main__":
    # If testing Reader independently, manual polling is required
    reader = MotorReader()
    
    try:
        print("Starting Inquiry Loop... Press Ctrl+C to stop.")
        while True:
            for motor_id in CAN_CONFIG.WHEEL_CAN_IDS:
                # Query speed
                reader.send_inquiry(motor_id, Protocol.QUERY_SPEED)
                time.sleep(0.01)
                # Query position
                reader.send_inquiry(motor_id, Protocol.QUERY_POS_ABS)
                time.sleep(0.01)
            
            # Display current status in the terminal once per second
            print(f"Current Speeds: {reader.cur_speeds}")
            time.sleep(1)

    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        reader.close()