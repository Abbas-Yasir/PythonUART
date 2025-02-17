import serial
import time
import math

print ("Bismillah")

import serial

def main():
    port = "ttyUSB0"  # Change this to your actual COM port
    baudrate = 9600  # Adjust as needed
    data_bits = 8
    stop_bits = 1
    parity = 'N'

    try:
        with serial.Serial(port, baudrate,bytesize=data_bits, stopbits=stop_bits, parity=parity, timeout=1) as ser:
            print(f"Connected to {port}")
            while True:
                if ser.in_waiting:
                    data = ser.read().decode('utf-8', errors='ignore')  # Read one byte
                    print(f"{data}", end='', flush=True)
    except serial.SerialException as e:
        print(f"Serial error: {e}")
    except KeyboardInterrupt:
        print("Program terminated.")

if __name__ == "__main__":
    main()
