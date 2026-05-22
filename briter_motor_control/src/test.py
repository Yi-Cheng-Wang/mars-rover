import time
import threading
import serial
import struct

# Parameter settings
SERIAL_PORT = 'COM6'
SERIAL_BAUDRATE = 2000000  # Default serial port baudrate for the USB to CAN module
CAN_BAUDRATE_CODE = 0x03   # 0x03 represents CAN baudrate 500kbps

# [Modification 1] Change a single ID to a list of IDs for multiple motors
TARGET_CAN_IDS = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06]  # Please modify according to the actual CAN IDs of your drives
HEARTBEAT_INTERVAL = 0.5   # Heartbeat transmission interval (seconds)

class MotorTestController:
    # [Modification 2] Pass in the motor ID list during initialization
    def __init__(self, port, baudrate, target_ids):
        self.target_ids = target_ids
        self.ser = serial.Serial(port=port, baudrate=baudrate, timeout=1, write_timeout=1)
        self.running = True
        
        # Add a thread lock to act as a traffic controller for serial port writing
        self.write_lock = threading.Lock()
        
        # 1. Initialize the CAN module (using variable-length protocol settings)
        self._init_can_module()
        
        # 2. Start the background heartbeat thread
        self.heartbeat_thread = threading.Thread(target=self._heartbeat_loop, daemon=True)
        self.heartbeat_thread.start()
        print(f"[System] Successfully connected and started the heartbeat thread (Monitoring IDs: {self.target_ids})...")

        # 3. Start the listening thread
        self.read_thread = threading.Thread(target=self._read_loop, daemon=True)
        self.read_thread.start()

    def _init_can_module(self):
        """Send a 20-byte setting command to initialize the USB-CAN adapter"""
        frame = [
            0xAA, 0x55, 0x12,
            CAN_BAUDRATE_CODE, # CAN baudrate (0x03 = 500k)
            0x01,              # Standard frame
            0x00, 0x00, 0x00, 0x00,  # Filter ID
            0x00, 0x00, 0x00, 0x00,  # Mask ID
            0x00,              # Normal mode
            0x01, 0x00, 0x00, 0x00, 0x00  # Reserved bytes
        ]
        checksum = sum(frame[2:19]) & 0xFF
        frame.append(checksum)
        
        with self.write_lock:
            self.ser.write(bytearray(frame))
            
        time.sleep(0.1) 
        print("[System] USB-CAN module initialization completed")

    def send_can_frame(self, can_id, data):
        """Encapsulate data into the variable-length protocol format of the USB-CAN module and send"""
        frame = bytearray()
        frame.append(0xAA)  
        
        # Type & DLC
        info_byte = 0xC0 | (len(data) & 0x0F)
        frame.append(info_byte)
        
        # CAN ID (2 bytes, LSB first)
        frame.append(can_id & 0xFF)
        frame.append((can_id >> 8) & 0xFF)
        
        # Data section
        frame.extend(data)
        
        # End code
        frame.append(0x55)
        
        try:
            with self.write_lock:
                self.ser.write(frame)
        except serial.SerialTimeoutException:
            print("[Warning] Serial port write timeout! The hardware might have lost response.")

    def _heartbeat_loop(self):
        """[Modification 3] Periodically send heartbeat packets to 'all' motors"""
        heartbeat_data = bytearray([0x00])
        while self.running:
            for can_id in self.target_ids:
                self.send_can_frame(can_id, heartbeat_data)
                time.sleep(0.005)  # Add a very short delay to avoid instantly overflowing the buffer of the USB to CAN module
            time.sleep(HEARTBEAT_INTERVAL)

    def set_speed(self, can_id, speed_erpm):
        """Set single motor speed (Command 0x02)"""
        data = bytearray([0x02])
        data.extend(int(speed_erpm).to_bytes(4, byteorder='big', signed=True))
        self.send_can_frame(can_id, data)
        # print(f"[Command] ID {can_id} set speed: {speed_erpm} erpm") # If it's too noisy, you can comment out this line

    def stop_motor(self, can_id):
        """Stop a single motor"""
        self.set_speed(can_id, 0)

    # [New] Auxiliary function to control all motors at once
    def set_all_speed(self, speed_erpm):
        """Set all motors to the same speed"""
        for can_id in self.target_ids:
            self.set_speed(can_id, speed_erpm)
            time.sleep(0.005) # Slightly stagger the transmission time
        print(f"[Command] All motors set to speed: {speed_erpm} erpm")

    def stop_all_motors(self):
        """Stop all motors"""
        for can_id in self.target_ids:
            self.stop_motor(can_id)
            time.sleep(0.005)
        print("[Command] All motors stopped")

    def close(self):
        self.running = False
        if self.heartbeat_thread.is_alive():
            self.heartbeat_thread.join()
        if self.read_thread.is_alive():
            self.read_thread.join()
        self.ser.close()
        print("[System] Serial port closed")

    def _read_loop(self):
        """Listen to the data returned by the USB-CAN module"""
        while self.running:
            try:
                if self.ser.in_waiting > 0:
                    if self.ser.read(1) == b'\xaa':
                        header_rest = self.ser.read(3) 
                        if len(header_rest) == 3:
                            dlc = header_rest[0] & 0x0F
                            can_id = header_rest[1] + (header_rest[2] << 8)
                            data = self.ser.read(dlc)
                            end_byte = self.ser.read(1)
                            
                            # hex_data = " ".join([f"{b:02X}" for b in data])
                            # print(f"\n[Receive] Received CAN ID: {can_id}, Data: {hex_data}")
            except Exception as e:
                if self.running:
                    print(f"[Warning] Exception occurred while reading the serial port: {e}")


if __name__ == '__main__':
    try:
        # Pass in the TARGET_CAN_IDS list
        motor_ctrl = MotorTestController(SERIAL_PORT, SERIAL_BAUDRATE, TARGET_CAN_IDS)
        
        print("\n--- Starting multi-motor test process ---")
        time.sleep(1) # Let the heartbeat be sent a few times first
        
        # Gradual acceleration (Forward - all motors simultaneously)
        for speed in range(0, 5100, 100):
            motor_ctrl.set_all_speed(speed)
            time.sleep(0.2)

        motor_ctrl.stop_all_motors()
        time.sleep(3)

        # Gradual acceleration (Reverse - all motors simultaneously)
        for speed in range(0, 5100, 100):
            motor_ctrl.set_all_speed(-speed)
            time.sleep(0.2)

        motor_ctrl.stop_all_motors()
        time.sleep(3)
        
        # Independent control example (if needed)
        # print("Testing independent control...")
        # motor_ctrl.set_speed(0x05, 1000)
        # motor_ctrl.set_speed(0x06, -1000)
        # time.sleep(3)
        # motor_ctrl.stop_all_motors()

        print("--- Test process ended ---\n")

    except serial.SerialException as e:
        print(f"\n[Error] Serial port operation failed: {e}")
        print(f"Please confirm that {SERIAL_PORT} is not occupied by other software, and check if the physical wiring is loose.")
    except KeyboardInterrupt:
        print("\n[System] User forced interruption of the test")
    finally:
        if 'motor_ctrl' in locals():
            motor_ctrl.stop_all_motors()
            motor_ctrl.close()