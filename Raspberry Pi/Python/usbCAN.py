import serial

port = serial.Serial("/dev/ttyUSB0", baudrate=115200, timeout=3.0)

def readlineCR(port):
    rv = ""
    while True:
        ch=port.read()
        rv+=ch
        if ch=='\r' or ch==' ':
            return rv

port.write("S6\r")
port.write("O\r")

while True:
    print(readlineCR(port))
