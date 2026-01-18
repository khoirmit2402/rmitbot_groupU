import serial

def readSerial(comport):
    ser = serial.Serial(comport, baudrate=115200)

    data = ser.readline()
    
    return data.decode()

if __name__ == '__main__':
    readSerial('/dev/scale')

