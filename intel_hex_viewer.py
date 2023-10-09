import sys
data = ""
def parse_hex_line( line ):
    if len( current_line ) == 0: return
    bytecount = int( line[0:2], 16 )
    address = int( line[2:6], 16 )
    rec_type = int( line[6:8], 16 )
    global data
    crc = int(line[-3:-1], 16)
    
    rec_output = str(hex(crc)) + '\t' + str(hex(address)) + '\t' + str(bytecount) + '\t'
    if rec_type == 0:
        rec_output += 'date'
        rec_output += '\t\t' + line[8 : (8 + 2 * (bytecount))]
        data += ( line[8 : (8 + 2 * (bytecount))] ) + "\n"
    elif rec_type == 1:
        rec_output += 'sfarsit fisier'
    elif rec_type == 2:
        rec_output += 'extindere adresa segment'
    elif rec_type == 3:
        rec_output += 'inceput adresa segment'
    elif rec_type == 4:
        rec_output += 'extindere adresa liniara'
    elif rec_type == 5:
        rec_output += 'inceput adresa liniara'
    print(rec_output)
    

#   Deschide fisierul hex
hex_file_path = sys.argv[1]
print(hex_file_path)
hex_file = open(hex_file_path, "r")

#   Analizare fisier linie cu linie
current_line = ""
try:
    byte = "1" # initial placeholder
    print ("CRC\tAdresa\tLungime\tTip\t\tDate")
    while byte != "":
        byte = hex_file.read(1)
        if byte == ":":
            #   Parsare linie curenta
            parse_hex_line( current_line )
            #   Resetare linie curenta pentru a o parsa pe urmatoarea
            current_line = ""
        else:
            current_line += byte
    parse_hex_line( current_line )
finally:
    hex_file.close()

fisier = open("led_data_only.hex","w")
fisier.write(data)
fisier.close()
