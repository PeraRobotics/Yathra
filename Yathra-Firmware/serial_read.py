import serial
import struct
import time

# --- CONFIGURATION ---
SERIAL_PORT = '/dev/ttyUSB0'  # CHECK THIS: Might be COM3 on Windows, /dev/ttyACM0 on Linux
BAUD_RATE = 115200

# --- STRUCT DEFINITIONS ---
# We must match the C++ struct exactly
# 1. Header: 8 bytes (char[8])
# 2. IMU Data: 
#    - Accel (3 floats: x,y,z)
#    - Gyro  (3 floats: x,y,z)
#    - Mag   (3 floats: x,y,z)
#    - Roll  (1 float)
#    - Pitch (1 float)
#    - Head  (1 float)
#    - Temp  (1 float)
# Total Floats: 3 + 3 + 3 + 1 + 1 + 1 + 1 = 13 floats
# Format String: < (Little Endian) 8s (String) 13f (Floats)

PACKET_FORMAT = '<8s13f'
PACKET_SIZE = struct.calcsize(PACKET_FORMAT)
TAG_TEL = b'SUB_TEL\0'

def read_serial():
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print(f"Connected to {SERIAL_PORT}...")
    except Exception as e:
        print(f"Error opening serial port: {e}")
        return

    buffer = b''

    while True:
        try:
            # Read data from serial
            chunk = ser.read(ser.in_waiting or 1)
            if not chunk:
                continue
            
            buffer += chunk

            # While we have enough data for a full packet
            while len(buffer) >= PACKET_SIZE:
                
                # Check for the Tag at the start
                # (We slice the first 8 bytes to compare)
                if buffer[0:8] == TAG_TEL:
                    
                    # Unpack the data
                    data = struct.unpack(PACKET_FORMAT, buffer[:PACKET_SIZE])
                    
                    # Parse tuple (data[0] is the tag)
                    # accel = (data[1], data[2], data[3])
                    # gyro  = (data[4], data[5], data[6])
                    roll  = data[10]
                    pitch = data[11]
                    head  = data[12]
                    temp  = data[13]

                    # Print formatted output
                    print(f"\r[SUB_TEL] Head: {head:6.1f} | Pitch: {pitch:6.1f} | Roll: {roll:6.1f} | Temp: {temp:4.1f}C", end='')
                    
                    # Remove the processed packet from buffer
                    buffer = buffer[PACKET_SIZE:]
                
                else:
                    # If tag not found, slide window by 1 byte to find the start
                    buffer = buffer[1:]
                    
        except KeyboardInterrupt:
            print("\nExiting...")
            ser.close()
            break
        except Exception as e:
            print(f"\nError: {e}")
            break

if __name__ == "__main__":
    read_serial()