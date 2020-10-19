import serial
import time

obj          = serial.Serial()
obj.baudrate = 9600
obj.port     = "/dev/cu.usbmodem14101"
obj.open()
obj.timeout = 3




obj.readline()
# Fetch sensor reading.
obj.write(bytearray([0xEA, 0xAD, 0x24, 0x98, 0X0A]))

#Write open valve.
#obj.write(bytearray([0xAC, 0xAE, 0X51, 0XA2, 0X02]))

incoming = obj.readline()
print(incoming)
for x in incoming:
    print(str(x))
