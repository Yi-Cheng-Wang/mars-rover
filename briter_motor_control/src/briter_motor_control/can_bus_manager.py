import serial
import threading
import time
from params import CAN_CONFIG, BriterMotorProtocol as Protocol

class CANBusManager:
    def __init__(self):
        self.write_lock = threading.Lock()
        self.callbacks = []  # Stores functions that need to receive CAN data
        
        try:
            self.ser = serial.Serial(
                port=CAN_CONFIG.PORT,
                baudrate=CAN_CONFIG.BAUDRATE,
                timeout=CAN_CONFIG.TIMEOUT,
                write_timeout=1
            )
            print(f"[CAN Manager] Successfully connected to {CAN_CONFIG.PORT}")
        except Exception as e:
            print(f"[CAN Manager] Serial port connection failed: {e}")
            exit(1)

        self.is_running = True
        
        # Start the globally unique receiving thread
        self.read_thread = threading.Thread(target=self._read_loop, daemon=True)
        self.read_thread.start()
        
        # Initialize USB-CAN adapter
        self.send_frame(Protocol.get_init_frame())
        time.sleep(0.1)
        print("[CAN Manager] USB-CAN module initialization complete.")

    def register_callback(self, callback):
        """Allow Reader to register a receiving function; this function will be called when Manager receives data"""
        self.callbacks.append(callback)

    def send_frame(self, frame):
        """A safe write method shared by Writer and Reader"""
        if not self.is_running:
            return
        try:
            with self.write_lock:
                self.ser.write(frame)
        except serial.SerialTimeoutException:
            pass # Prevent terminal spam

    def _read_loop(self):
        """Globally unique read loop, responsible for parsing packets and distributing to Callbacks"""
        while self.is_running:
            try:
                if self.ser.in_waiting > 0:
                    # Look for packet header 0xAA
                    if self.ser.read(1) == b'\xAA':
                        info = self.ser.read(1)
                        if not info: continue
                        dlc = info[0] & 0x0F

                        # Read CAN ID
                        id_bytes = self.ser.read(2)
                        if len(id_bytes) < 2: continue
                        can_id = (id_bytes[1] << 8) | id_bytes[0]

                        # Read Payload
                        payload = list(self.ser.read(dlc))

                        # Check footer 0x55
                        footer = self.ser.read(1)
                        if footer == b'\x55':
                            # Distribute parsed data to all registered Readers
                            for cb in self.callbacks:
                                cb(can_id, payload)
            except Exception as e:
                if self.is_running:
                    print(f"[CAN Manager] Packet parsing error: {e}")
            
            time.sleep(0.001)

    def close(self):
        """Close the manager and serial port"""
        self.is_running = False
        if self.read_thread.is_alive():
            self.read_thread.join(timeout=1.0)
        self.ser.close()
        print("[CAN Manager] Serial port has been safely closed.")