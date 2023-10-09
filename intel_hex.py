from intelhex import IntelHex
ih = IntelHex("led_salt2.hex")
ih.tofile("led_salt2.bin", format = 'bin')

