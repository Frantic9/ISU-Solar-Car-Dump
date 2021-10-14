import serial

port = serial.Serial("/dev/ttyUSB0", baudrate=115200, timeout=3.0)

def readlineCR(port):
    rv = ""
    while True:
        ch=port.read()
        rv+=ch
        if ch=='\r' or ch==' ':
            return rv

while True:
    port.write(raw_input("Please inter can frame in format TiiiLDD...."))
    port.write("\r")
