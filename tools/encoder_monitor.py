import serial
import threading

port = "/dev/cu.usbmodem1101"

with serial.Serial(port, 115200, timeout=0.2) as ser:
    print("Connected! Press e to mark encoder positions.")
    print("Press x to STOP. Press q to quit.")

    running = True

    def read_serial():
        with open("encoder_log.txt", "a") as log:
            while running:
                line = ser.readline().decode(errors="replace").strip()
                if line:
                    log.write(line + "\n")
                    log.flush()
                    if any(k in line for k in (
                        "SCAN", "Obstacle", "SERVO TEST",
                        "Encoder", "STOP"
                    )):
                        print(line, flush=True)

    thread = threading.Thread(target=read_serial, daemon=True)
    thread.start()

    try:
        while True:
            key = input("Command (e/x/q): ").strip().lower()
            if key == "q":
                break
            if key in ("e", "x", "p", "t", "g"):
                ser.write(key.encode())
                ser.flush()
    finally:
        running = False
        thread.join(timeout=1)
