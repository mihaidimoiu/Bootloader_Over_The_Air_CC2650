import serial

s = serial.Serial(port = "COM5",baudrate = 9600,bytesize = 8,stopbits = 1)
s.write(bytes([0xAA,0x01]))
s.close()
