import serial
import time
import math

print ("Bismillah")

# Function to convert NMEA lat/lon to degrees
def nmea2deg(nmea_value):
    degrees = int(nmea_value / 100)
    minutes = nmea_value - degrees * 100
    return degrees + minutes / 60

# Function to convert degrees to radians
def deg2rad(degrees):
    return math.radians(degrees)

# Function to convert latitude, longitude, and altitude to ECEF (Earth-Centered Earth-Fixed) coordinates
def lla2ecef(lat, lon, alt):
    # WGS84 ellipsoid constants
    a = 6378137.0
    e = 8.1819190842622e-2

    # Prime vertical radius of curvature
    N = a / math.sqrt(1 - e**2 * math.sin(lat)**2)

    # ECEF coordinates
    x = (N + alt) * math.cos(lat) * math.cos(lon)
    y = (N + alt) * math.cos(lat) * math.sin(lon)
    z = ((1 - e**2) * N + alt) * math.sin(lat)

    return x, y, z

def parse_nmea():
    # Define Serial Port parameters
    port = 'ttyUSB0'  # Set the appropriate port
    baud_rate = 115200
    data_bits = 8
    stop_bits = 1
    parity = 'N'

    # Open serial port connection
    s = serial.Serial(port, baud_rate, bytesize=data_bits, stopbits=stop_bits, parity=parity)
    s.timeout = 2  # Timeout for reading

    # Command to switch to NMEA
    set2nmea = bytes([0xA0, 0xA1, 0x00, 0x03, 0x09, 0x01, 0x00, 0x08, 0x0D, 0x0A])

    # Send the command
    s.write(set2nmea)

    ack_sw = [0xA0, 0xA1, 0x00, 0x02, 0x83, 0x09, 0x8A, 0x0D, 0x0A]

    # Wait for acknowledgment
    response = s.read(9)
    if list(response) == ack_sw:
        print("NMEA switch successful")
    else:
        print("NMEA Switch Failed")

    read_duration = 120  # Read duration in seconds

    utc = None
    prn_str = None

    start_time = time.time()

    while time.time() - start_time < read_duration:
        if s.in_waiting:
            output = s.readline().decode('ascii', errors='replace').strip()
            
            # Check for GPGGA sentence
            if output.startswith('$GPGGA'):
                parts = output.split(',')
                if len(parts) > 10:
                    # Extract UTC and altitude information
                    utc_time = parts[1]
                    if utc_time:
                        hh = int(utc_time[0:2])
                        mm = int(utc_time[2:4])
                        ss = int(utc_time[4:6])
                        sss = int(utc_time[7:]) if len(utc_time) > 6 else 0
                        utc = f"{hh:02d}:{mm:02d}:{ss:02d}.{sss:03d}"
                    
            # Check for GNGSA sentence
            if output.startswith('$GNGSA'):
                parts = output.split(',')
                if len(parts) > 15:
                    prn_s4f = [p for p in parts[3:15] if p]
                    prn_s4f = [int(p) for p in prn_s4f if p.isdigit()]
                    if prn_s4f:
                        prn_str = ', '.join(map(str, prn_s4f))
            
            # If both values are found, break out of the loop
            if utc and prn_str:
                break

    # Close serial port
    s.close()
    # print(f"prn_s4f={prn_s4f}")

    return utc, prn_s4f

# Example of usage
utc, prn_s4f = parse_nmea()
print(f"UTC Time: {utc}")
print(f"PRN Satellites used for fix: {prn_s4f}")
# print(prn_s4f[0])