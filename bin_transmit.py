import serial
import time
import math
import os

def receive_chunk(size):
    chunk = list(range(size))

    for i in range(size):
        chunk[i] = ser.read().hex()

    return chunk

def chunk_to_str(chunk):
    return "".join(chunk)

ser = serial.Serial(port="/dev/tty.usbmodem12303", baudrate=115200)
ser.timeout = 1

fileName = "Blink.bin"

adress = 0x20 #as a page number

size = 0x0

chunk_size = 64
sent = 0
received = 0

RECEIVE_ADRESS_CMD = 0xFF
RECEIVE_SIZE_CMD = 0xFA
RECEIVE_DATA_CMD = 0xAF
FINISH_UPDATE_CMD = 0xAA

file_stats = os.stat(fileName)
size = math.ceil(file_stats.st_size / 1024)
print(file_stats.st_size)
print("File size: " + str(size) + "KB")
size = math.ceil(size / 2)
print("Pages: " + str(size))


ser.write(RECEIVE_ADRESS_CMD.to_bytes())
ser.write(adress.to_bytes())
ser.write(RECEIVE_SIZE_CMD.to_bytes())
ser.write(size.to_bytes())
time.sleep(1)

with open(fileName, "rb") as f:
    while True:
        chunk = f.read(chunk_size)
        if chunk:
            if len(chunk) < chunk_size:
                print("Old chunk size: " + str(len(chunk)))
                chunk = bytearray(chunk)

                chunk[:] += bytes(chunk_size - len(chunk))
                chunk = bytes(chunk)

                print(chunk)
                print("New chunk size: " + str(len(chunk)))
            
            if len(chunk) == chunk_size:
                ser.write(RECEIVE_DATA_CMD.to_bytes())
                for b in chunk:
                    ser.write(b.to_bytes())

                #time.sleep(0.1)
                sent += len(chunk)

                value = receive_chunk(chunk_size)

                value = chunk_to_str(value)
                print(value)
                received += len(value) / 2

        else:
            f.close()
            break
    
    ser.write(FINISH_UPDATE_CMD.to_bytes())

    print("Total sent: " + str(sent))
    print("Total received: " + str(received))

