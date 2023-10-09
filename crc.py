import serial
import time
import os
'''s = serial.Serial(port = "COM4",baudrate = 9600,bytesize = 8,stopbits = 1)
filename_bin = "led2.bin"
size_app = os.path.getsize(filename_bin)

x = (size_app).to_bytes(4, 'big')

start_time = time.time()

s.write(bytes([0x18]))
    
elapsed_time = time.time() - start_time
    
print(elapsed_time)
s.close()'''
#f.seek(pozitia unde vrei sa te pozitionezi, 0=absoult adica de la inceput/ 1 = relativ la pozitia actuala)
filename_bin = "led1.bin"
'''f = open(filename_bin, "rb")
print(f.read(20))
f.seek(0,0)
print(f.read(10))
f.seek(-2,1)
print(f.read(10))
f.close()

'''
s = serial.Serial(port = "COM5",baudrate = 9600,bytesize = 8,stopbits = 1)
import binascii

def CRC32_from_file(filename_bin):
    buf = open(filename_bin,'rb').read()
    buf = (binascii.crc32(buf) & 0xFFFFFFFF)
    return buf
# return "%08X" % buf converteste in hex
'''x = CRC32_from_file(filename_bin)
print(x)
print(x.bit_length())
print(x.to_bytes(4,'big'))'''

cerece = binascii.crc32(bytes([0x80,0x10])) & 0xFFFFFFFF
print(cerece)
s.write(bytes([0x80,0x10]))
s.write(cerece)
s.read(3)

#cerece = "%08X" % cerece
#print(cerece)

s.close()
