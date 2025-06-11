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

# ── reader thread ───────────────────────────────────────────
# def read_thread():
#     while True:
#         line = ser.readline().decode('ascii', errors='ignore')#.strip()
#         if line:
#             print(line)

# t = threading.Thread(target=read_thread, daemon=True)
# t.start()

# ── send helper ─────────────────────────────────────────────
def send(motor_id, cmd, val):
    msg = f"{motor_id},{cmd},{val}\n"
    ser.write(msg.encode('ascii'))

# ── example ─────────────────────────────────────────────────
if __name__ == "__main__":
    send(0, 'setMode', 3)
    time.sleep(0.1)  # wait for Teensy to process the command
    send(0, 'setDuty', 1000)


