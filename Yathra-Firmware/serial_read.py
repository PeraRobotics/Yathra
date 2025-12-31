import serial
import struct
import time
import threading
import sys

# --- Configuration ---
SERIAL_PORT = '/dev/ttyUSB0'  # Change to COMx on Windows
BAUD_RATE = 115200

# --- Protocol Definitions ---
SYNC_BYTE = 0xA5

# Packet IDs (Must match C enum)
TEL_ID_IMU   = 0x10
TEL_ID_AHRS  = 0x11
TEL_ID_BARO  = 0x12
CMD_ID_CONFIG = 0x20
CMD_ID_TARGET = 0x21

# Struct Formats (Little Endian '<')
# f = float (4 bytes), ? = bool (1 byte)
# IMU: 3 accel + 3 gyro + 3 mag = 9 floats
FMT_IMU   = '<9f' 
# AHRS: heading, pitch, roll = 3 floats
FMT_AHRS  = '<3f'
# BARO: press_out, press_in, temp = 3 floats
FMT_BARO  = '<3f'
# CONFIG: arm(bool), kp, ki, kd (floats), 6 motor_rev (bools)
# Note: 6s is 6 bytes string, but for individual bools we use 6?
FMT_CONFIG = '<?3f6?' 
# TARGET: v, w, h (3 floats)
FMT_TARGET = '<3f'

class RobotLink:
    def __init__(self, port, baud):
        try:
            self.ser = serial.Serial(port, baud, timeout=1)
            self.running = True
            print(f"Connected to {port} at {baud}")
        except serial.SerialException as e:
            print(f"Error connecting to serial: {e}")
            sys.exit(1)

    def start_listener(self):
        """Starts the RX thread to print incoming data"""
        self.rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self.rx_thread.start()

    def _rx_loop(self):
        print("Listening for telemetry...")
        while self.running:
            try:
                # 1. Read one byte to find SYNC
                if self.ser.read(1) == bytes([SYNC_BYTE]):
                    # 2. Read ID
                    packet_id_byte = self.ser.read(1)
                    if not packet_id_byte: continue
                    
                    packet_id = packet_id_byte[0]

                    # 3. Decode based on ID
                    # if packet_id == TEL_ID_IMU:
                    #     data = self.ser.read(struct.calcsize(FMT_IMU))
                    #     if len(data) == struct.calcsize(FMT_IMU):
                    #         # Unpack all 9 floats
                    #         ax, ay, az, gx, gy, gz, mx, my, mz = struct.unpack(FMT_IMU, data)
                            
                    #         # Print all of them
                    #         print(f"[IMU] A: {ax:.2f} {ay:.2f} {az:.2f} | G: {gx:.2f} {gy:.2f} {gz:.2f} | M: {mx:.2f} {my:.2f} {mz:.2f}")
                    
                    # if packet_id == TEL_ID_AHRS:
                    #     data = self.ser.read(struct.calcsize(FMT_AHRS))
                    #     if len(data) == struct.calcsize(FMT_AHRS):
                    #         h, p, r = struct.unpack(FMT_AHRS, data)
                    #         print(f"[AHRS] H: {h:.1f} P: {p:.1f} R: {r:.1f}")

                    if packet_id == TEL_ID_BARO:
                        data = self.ser.read(struct.calcsize(FMT_BARO))
                        if len(data) == struct.calcsize(FMT_BARO):
                            p_out, p_in, temp = struct.unpack(FMT_BARO, data)
                            print(f"[BARO] Press: {p_out:.1f} Temp: {temp:.1f}")

            except Exception as e:
                print(f"RX Error: {e}")

    def send_config(self, arm, kp, ki, kd, motor_revs):
        """
        arm: bool
        kp, ki, kd: floats
        motor_revs: list of 6 bools [False, True, ...]
        """
        payload = struct.pack(FMT_CONFIG, arm, kp, ki, kd, *motor_revs)
        self._send_packet(CMD_ID_CONFIG, payload)
        print(f"-> Sent Config: Arm={arm}, PID={kp}/{ki}/{kd}")

    def send_target(self, v, w, h):
        """
        v: velocity (float)
        w: angular vel (float)
        h: height (float)
        """
        payload = struct.pack(FMT_TARGET, v, w, h)
        self._send_packet(CMD_ID_TARGET, payload)
        # print(f"-> Sent Target: v={v} w={w} h={h}")

    def _send_packet(self, cmd_id, payload):
        # Header: SYNC + ID
        header = bytes([SYNC_BYTE, cmd_id])
        self.ser.write(header + payload)

    def close(self):
        self.running = False
        self.ser.close()

# --- Main Test Logic ---
if __name__ == "__main__":
    link = RobotLink(SERIAL_PORT, BAUD_RATE)
    link.start_listener()

    try:
        # 1. Send Initial Config
        # Arm=True, PID=(1.5, 0.02, 0.5), Motors=[F, F, F, F, F, F]
        time.sleep(2)
        link.send_config(True, 1.5, 0.02, 0.5, [False]*6)

        # 2. Send Changing Targets
        t = 0
        while True:
            # Send target every 100ms
            # Example: Sine wave height
            import math
            target_h = 10.0 + 5.0 * math.sin(t)
            
            link.send_target(v=1.0, w=0.0, h=target_h)
            
            time.sleep(0.1)
            t += 0.1

    except KeyboardInterrupt:
        print("\nStopping...")
        link.close()