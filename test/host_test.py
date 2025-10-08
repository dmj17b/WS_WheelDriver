import serial
import serial.tools.list_ports
import threading
import time

VENDOR_ID  = 0x16C0
PRODUCT_ID = 0x0483

def find_teensy_port(vid, pid):
    """
    Scan all serial ports and return the device name
    whose .vid/.pid match your Teensy.
    """
    for p in serial.tools.list_ports.comports():
        if p.vid == vid and p.pid == pid:
            return p.device
    raise IOError(f"Teensy (VID=0x{vid:04X}, PID=0x{pid:04X}) not found")

# ── open the port ───────────────────────────────────────────
port = find_teensy_port(VENDOR_ID, PRODUCT_ID)
ser  = serial.Serial(port, 115200, timeout=0.1)
print(f"→ Connected to Teensy on {port}")

def read_from_port(ser):
    """
    Read and parse streaming data in format (int,float) from serial port.
    Returns parsed data as tuple or None if parsing fails.
    """
    while True:
        line = ser.readline().decode('ascii').strip()
        if line:
            # print(f"← Raw: {line}")
            
            # Parse the (int,float) format
            parsed_data = parse_serial_data(line)
            if parsed_data:
                int_value, float_value = parsed_data
                print(f"→ Parsed: int={int_value}, float={float_value}")
                

def parse_serial_data(line):
    """
    Parse a line in format int,float and return tuple of (int, float).
    Returns None if parsing fails.
    
    Examples:
    - "123,45.67" -> (123, 45.67)
    - "0,-12.34" -> (0, -12.34)
    - "-5,3.14159" -> (-5, 3.14159)
    """
    try:
        # Remove whitespace and split by comma
        line = line.strip()
        if not line:  # Skip empty lines
            return None
        
        parts = line.split(',')
        
        if len(parts) != 2:
            return None
        
        # Parse int and float
        int_part = int(parts[0].strip())
        float_part = float(parts[1].strip())
        
        return (int_part, float_part)
        
    except (ValueError, IndexError) as e:
        print(f"Parse error: {e} for line: '{line}'")
        return None



# ── send helper ─────────────────────────────────────────────
def send(motor_id, cmd, val):
    msg = f"{motor_id},{cmd},{val}\n"
    ser.write(msg.encode('ascii'))

# ── example ─────────────────────────────────────────────────
if __name__ == "__main__":
    send(8, 'setKp', 20)
    send(8, 'setKi', 10)
    send(8, 'setVel', 3)
    read_from_port(ser)
    time.sleep(5)
    send(8, 'setVel', 0)


