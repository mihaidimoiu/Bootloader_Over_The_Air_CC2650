# -*- coding: utf-8 -*-
import serial
import time
import os
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

#def getBytes(filename, num):
#    with open(filename, "rb") as f:
#        byte = f.read(num)
#        return byte
    
#bytee = getBytes(filename_bin, -1)

#s.write(bytee)

size_app = os.path.getsize(filename_bin)
pbar = ProgressBar(widgets=widgets, maxval=size_app)
x = (size_app).to_bytes(4, 'big')
print("Scriere " + filename_bin)
print("Dimensiune " + str(size_app))

start_time = time.time()

s.write(bytes([0x19]))
s.write(bytes([0x00, 0x01, 0x0, 0x00]))
s.write(x)

dim = 0
i = 0
pbar.start()
with open(filename_bin, "rb") as f:
    byte = f.read(80)
    dim = len(byte)
    s.write(bytes([0x29]))
    s.write(bytes([dim]))
    s.write(byte)
    i += dim
    pbar.update(i)
    while byte:
        # Do stuff with byte.
        byte = f.read(80)
        dimm = len(byte)
        s.write(bytes([0x29]))
        s.write(bytes([dim]))
        s.write(byte)
        time.sleep(0.1)
        i += dim
        pbar.update(i)
pbar.finish()
elapsed_time = time.time() - start_time
    
print(elapsed_time)


s.close()
