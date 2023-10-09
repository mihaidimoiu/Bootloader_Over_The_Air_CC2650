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
filename_bin = "led_salt2.bin"
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
s.write(bytes([0xAA]))
s.write(bytes([0x10]))
s.write(bytes([0x00, 0x01, 0x00, 0x00]))
s.write(x)

raspuns = s.read(1)
if raspuns == bytes([0x09]):
    #s.write(bytes([0x19]))
    print("ack")


dim = 0
i = 0
pbar.start()
with open(filename_bin, "rb") as f:
    byte = f.read(80)
    dim = len(byte)
    s.write(bytes([0xAA]))
    s.write(bytes([0x11]))
    s.write(bytes([dim]))
    s.write(byte)
    raspuns = s.read(1)
    if raspuns == bytes([0x55]):
        i += dim
        pbar.update(i)
    else:
        print("error1")
    while byte:
        byte = f.read(80)
        dim = len(byte)
        s.write(bytes([0xAA]))
        s.write(bytes([0x11]))
        s.write(bytes([dim]))
        s.write(byte)
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

