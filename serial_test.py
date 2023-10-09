# -*- coding: utf-8 -*-
'''import serial
import time
import os

start_time = time.time()

s = serial.Serial(port = "COM4",baudrate = 9600,bytesize = 8,stopbits = 1)

filename_bin = "led1.bin"
filename_hex = "led1.hex"

def getLines(filename):
    return open(filename).read().splitlines()

lines = getLines(filename_hex)

def getBytes(filename, num):
    with open(filename, "rb") as f:
        byte = f.read(num)
        return byte
    
bytee = getBytes(filename_bin, -1)
size_app = os.path.getsize(filename_bin)
print(size_app)
x = (size_app).to_bytes(4, 'big')

s.write(bytes([0x10]))

s.write(bytes([0x00, 0x01, 0x0, 0x00]))

s.write(x)

raspuns = s.read(1)
print(raspuns)
if raspuns == bytes([0x09]):
    s.write(bytes([0x19]))
    print("ack")

#time.sleep(0.2)
#s.write(bytee)

s.close()
elapsed_time = time.time() - start_time
    
print(elapsed_time)'''

import serial
import time
import os
import binascii
from progressbar import Bar, AdaptiveETA, FileTransferSpeed, Percentage, ProgressBar
class CrazyFileTransferSpeed(FileTransferSpeed):
    def update(self, pbar):
        if 45 < pbar.percentage() < 80:
            return FileTransferSpeed.update(self,pbar)
        else:
            return FileTransferSpeed.update(self,pbar)


widgets = [CrazyFileTransferSpeed(), Bar(),
               Percentage(),' ', ' ', AdaptiveETA()]

s = serial.Serial(port = "COM5",baudrate = 9600,bytesize = 8,stopbits = 1)
filename_bin = "led2.bin"
size_app = os.path.getsize(filename_bin)
pbar = ProgressBar(widgets=widgets, maxval=size_app)
x = (size_app).to_bytes(4, 'big')

print("Scriere " + filename_bin)
print("Dimensiune " + str(size_app))

start_time = time.time()


#s.write(bytes([0x30]))
#print("Testare conexiune")
#time.sleep(1)
#s.write(bytes([0x19, 0x19]))
#print("Trimitere adresa")

#time.sleep(2)

s.write(bytes([0x10]))
s.write(bytes([0x00, 0x01, 0x00, 0x00]))
s.write(x)

raspuns = s.read(1)
if raspuns == bytes([0x09]):
    #s.write(bytes([0x19]))
    print("ack")
#time.sleep(0.5)


dim = 0
i = 0
pbar.start()
with open(filename_bin, "rb") as f:
    byte = f.read(80)
    dim = len(byte)
    s.write(bytes([0x11]))
    s.write(bytes([dim]))
    s.write(byte)
    crc = binascii.crc32(byte)
    s.write(crc)
    #time.sleep(0.5)
    raspuns = s.read(1)
    if raspuns == bytes([0x55]):
        i += dim
        pbar.update(i)
    else:
        print("error1")
    while byte:
        byte = f.read(80)
        dim = len(byte)
        s.write(bytes([0x11]))
        s.write(bytes([dim]))
        s.write(byte)
        crc = binascii.crc32(byte)
        s.write(crc)
        raspuns = s.read(1)
        if raspuns == bytes([0x55]):
            i += dim
            pbar.update(i)
        else:
            print("error2")
    
elapsed_time = time.time() - start_time
pbar.finish()
print(elapsed_time)
s.close()

