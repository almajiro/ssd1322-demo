from PIL import Image, ImageFont, ImageDraw, ImageSequence
import numpy as np
import serial
import time
import sys
import cv2
from mss import mss

from mpd import MPDClient

CONTROL_WORD = 0x04
COMMAND_WORD = 0x02
WRITE_START_WORD = 0xE0
WRITE_END_WORD = 0xF0

def notifications(bus, message):
    print('get notify')

def send_byte(byte):
    device.write(bytes([byte & 0xFF]))

def send_data(byte):
#    send_byte(0xE4)
    send_byte(byte)
#    send_byte(WRITE_END_WORD)

def send_command(byte):
    send_byte(0x06)
    send_byte(byte)

def val_map(value, in_min, in_max, out_min, out_max):
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def sec_to_min(value):
    minute = 0
    while (value >= 60):
        value -= 60
        minute += 1
    return '{:02}'.format(minute) + ':' + '{:02}'.format(value)

def send_buffer(bitmap):
    send_byte(0xE4)
    for row in bitmap:

        for col in range(0, len(row), 2):
            data = 0x00

            if row[col]:
                data |= int(val_map(row[col], 0, 256, 0, 15))  << 4
        
            if row[col+1]:
                data |= int(val_map(row[col+1], 0, 256, 0, 15))
        
            send_data(data)



#    send_byte(0x06)

def grayscale(img):
    rgb = np.array(img, dtype="float32");
    
    rgbL = pow(rgb/255.0, 2.2)
    r, g, b = rgbL[:,:,0], rgbL[:,:,1], rgbL[:,:,2]
    grayL = 0.299 * r + 0.587 * g + 0.114 * b  # BT.601
    gray = pow(grayL, 1.0/2.2)*255

    return gray

#device = serial.Serial('/dev/ttyUSB2', 2000000)
#device = serial.Serial('/dev/ttyUSB1', 1209600, serial.EIGHTBITS, serial.PARITY_EVEN)
device = serial.Serial('/dev/ttyUSB0', 2000000, serial.EIGHTBITS, serial.PARITY_EVEN)
#device = serial.Serial('/dev/ttyUSB1', 115200, serial.EIGHTBITS, serial.PARITY_EVEN)
#device = serial.Serial('/dev/ttyUSB1', 2000000)

mpd = MPDClient()
mpd.connect('localhost', 6600)

fontMid = ImageFont.truetype('./fonts/DBSTRAIG.TTF', 12)
albumFont = ImageFont.truetype('./fonts/851Gkktt_005.ttf', 12)
faFont = ImageFont.truetype('/usr/share/fonts/TTF/fa-solid-900.ttf', 12)

fill = (255, 255, 255)
fillInverse = (0, 0, 0)

mon = {'top': 2520, 'left': 20, 'width': 1050, 'height': 400}
sct = mss()

while True:
    sct_img = sct.grab(mon)
    video = Image.frombytes('RGB', sct_img.size, sct_img.bgra, "raw", "BGRX")
    #video = Image.open('/tmp/cover.jpg')
    #cv2.imshow('test', np.array(video))

    mini_video = video.copy()

    img = Image.new('RGB', (256, 64))
    img.paste(mini_video.resize((256,64)), (0, 0))
    send_buffer(grayscale(img))

    #time.sleep(0.005)
    time.sleep(0.008)
    #time.sleep(0.01)
    #time.sleep(5)
    #print("next")

device.close()
