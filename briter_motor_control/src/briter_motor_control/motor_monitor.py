import time
import threading
import logging
from params import CAN_CONFIG, SAFETY_THRESHOLD, BriterMotorProtocol as Protocol
from can_writer import MotorController
from can_reader import MotorReader

# Configure logging system
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(message)s',
    handlers=[logging.FileHandler("system_monitor.log"), logging.StreamHandler()]
)

class VehicleMonitor:
    def __init__(self, controller: MotorController):
        self.controller = controller
        # Share the Serial object to avoid port conflicts
        self.reader = MotorReader(shared_serial=controller.ser)
        
        self.is_monitoring = True
        self.safety_status = {can_id: {"fault": 0, "temp": 0, "volt": 0, "curr": 0} 
                             for can_id in CAN_CONFIG.WHEEL_CAN_IDS}
        
        # Start the main monitoring logic thread
        self.monitor_thread = threading.Thread(target=self._monitor_execution, daemon=True)
        self.monitor_thread.start()

    def _monitor_execution(self):
        """ Periodically loop to query critical safety data """
        while self.is_monitoring:
            for can_id in CAN_CONFIG.WHEEL_CAN_IDS:
                # 1. Query fault code (highest priority)
                self.reader.send_inquiry(can_id, Protocol.QUERY_FAULT)
                time.sleep(0.02)
                
                # 2. Query temperature and voltage
                self.reader.send_inquiry(can_id, Protocol.QUERY_TEMP)
                time.sleep(0.02)
                self.reader.send_inquiry(can_id, Protocol.QUERY_VOLTAGE)
                time.sleep(0.02)
                
                # 3. Query current
                self.reader.send_inquiry(can_id, Protocol.QUERY_CURRENT_MOTOR)
                time.sleep(0.02)
                
                # Perform data classification evaluation
                self._evaluate_safety(can_id)
                
            time.sleep(0.1)

    def _evaluate_safety(self, can_id):
        """ Data classification processing logic """
        # Get the latest parsed data from the reader's cache
        # Assumption: the reader stores parsed values into its internal cache
        
        # Logic demonstration for classification processing
        index = CAN_CONFIG.WHEEL_CAN_IDS.index(can_id)
        
        # Get the currently parsed values from the reader
        # Note: In actual implementation, reader._handle_payload should update safety_status here
        current_data = self.reader.cur_data_cache[can_id] # Assuming reader has this cache structure

        # --- LEVEL 3: CRITICAL (Trigger emergency brake) ---
        reason = None
        if current_data['fault'] != 0:
            reason = f"Motor Fault Code: {hex(current_data['fault'])}"
        elif current_data['temp'] >= SAFETY_THRESHOLD.MAX_TEMP_CRITICAL:
            reason = f"Overheat Critical: {current_data['temp']}C"
        elif current_data['volt'] <= SAFETY_THRESHOLD.MIN_VOLTAGE:
            reason = f"Low Voltage Critical: {current_data['volt']}V"
        elif current_data['curr'] >= SAFETY_THRESHOLD.MAX_CURRENT_CRITICAL:
            reason = f"Overcurrent Critical: {current_data['curr']*10}mA"

        if reason:
            logging.error(f"CRITICAL STOP on ID {can_id:X}: {reason}")
            self.controller.emergency_brake()
            return

        # --- LEVEL 2: WARNING (Log and alert only) ---
        if current_data['temp'] >= SAFETY_THRESHOLD.MAX_TEMP_WARNING:
            logging.warning(f"ID {can_id:X} Temperature High: {current_data['temp']}C")
        
        if current_data['curr'] >= SAFETY_THRESHOLD.MAX_CURRENT_WARNING:
            logging.warning(f"ID {can_id:X} Load High: {current_data['curr']*10}mA")

        # --- LEVEL 1: INFO (Regular data recording) ---
        # Speed and position data are automatically recorded to CSV by the reader class

    def stop_monitoring(self):
        self.is_monitoring = False
        self.monitor_thread.join()
        logging.info("Monitor system shutdown.")

# ==========================================
# Full Integration Test
# ==========================================
if __name__ == "__main__":
    # 1. Start the controller
    ctrl = MotorController()
    
    # 2. Start the monitor and pass the controller instance
    monitor = VehicleMonitor(ctrl)
    
    try:
        logging.info("System Integrated. Starting Mission...")
        
        # Simulate normal driving
        # Set acceleration and target speed
        for motor_id in CAN_CONFIG.WHEEL_CAN_IDS:
            ctrl.ser.write(Protocol.pack_command_frame(motor_id, Protocol.CMD_SET_ACCEL_SPEED, 500))
            ctrl.ser.write(Protocol.pack_command_frame(motor_id, Protocol.CMD_SET_SPEED, 1000))
        
        # Keep main program running while the monitor thread guards safety
        while True:
            time.sleep(1)
            
    except KeyboardInterrupt:
        logging.info("Mission interrupted by operator.")
    finally:
        monitor.stop_monitoring()
        ctrl.close()