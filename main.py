'''''''''''''''''''''''''''''''''''''''''''''''''''''''''
Program contains elements of code from:

#   https://github.com/AnbuKumar-maker/ESP32-ESP32CAM
    by Anbu Kumar

#   https://github.com/miketeachman/micropython-i2s-examples/blob/master/examples/play_tone.py
    Copyright (c) 2022 Mike Teachman - The MIT License (MIT)
    
#   https://github.com/2black0/MicroPython-ESP32-BLE/blob/main/main.py
    Copyright (c) 2021 Ardy Seto - The MIT License (MIT)
    
#   https://github.com/glenn20/micropython/blob/espnow-g20-dev/docs/library/espnow.rst#id5
    by glenn20
    
#   https://github.com/zhcong/ULN2003-for-ESP32
    by zhcong
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''

from machine import Pin, I2S, Timer, PWM
import time
from time import sleep_ms
import hcsr04
import Stepper
import os
import math
import struct
import network
from esp import espnow
import ubinascii
import _thread



#-------------------------------------------------------------------
#===========================   SETUP   =============================
#-------------------------------------------------------------------


# RGB diode pins definitions
# --- GARAGE
LED_R_PIN_GARAGE = 25
LED_G_PIN_GARAGE = 27
LED_B_PIN_GARAGE = 26
# --- OFFICE
LED_R_PIN_OFFICE = 14
LED_G_PIN_OFFICE = 33
LED_B_PIN_OFFICE = 32
# --- BATHROOM
LED_R_PIN_BATHROOM = 21
LED_G_PIN_BATHROOM = 22
LED_B_PIN_BATHROOM = 23

# setting diode pins
# --- GARAGE
ledR_g = Pin(LED_R_PIN_GARAGE, Pin.OUT)
ledG_g = Pin(LED_G_PIN_GARAGE, Pin.OUT)
ledB_g = Pin(LED_B_PIN_GARAGE, Pin.OUT)
# --- OFFICE
ledR_o = Pin(LED_R_PIN_OFFICE, Pin.OUT)
ledG_o = Pin(LED_G_PIN_OFFICE, Pin.OUT)
ledB_o = Pin(LED_B_PIN_OFFICE, Pin.OUT)
# --- BATHROOM
ledR_b = Pin(LED_R_PIN_BATHROOM, Pin.OUT)
ledG_b = Pin(LED_G_PIN_BATHROOM, Pin.OUT)
ledB_b = Pin(LED_B_PIN_BATHROOM, Pin.OUT)


# stepper motor pins definitions
IN1_PIN = 15
IN2_PIN = 0
IN3_PIN = 4
IN4_PIN = 16

# setting stepper engine
s1 = Stepper.create(Pin(IN1_PIN,Pin.OUT),Pin(IN2_PIN,Pin.OUT),Pin(IN3_PIN,Pin.OUT),Pin(IN4_PIN,Pin.OUT), delay=1)

# distance meter pins definitions
TRIGGER_PIN = 13
ECHO_PIN = 12

# I2S amplifier pins definitions
AMP_SHUTDOWN_PIN = 17    # SD (shut down) pin
AMP_SD_PIN = 5     # DIN on board (serial data)
AMP_SCK_PIN = 18   # DCLK on board
AMP_WS_PIN = 19    # LRCLK on board

# setting distance meter paramethers
distance_meter = hcsr04.HCSR04(TRIGGER_PIN, ECHO_PIN, echo_timeout_us=1000000)

# I2S configuration
I2S_ID = 0
BUFFER_LENGTH_IN_BYTES = 2000

# audio configuration
TONE_FREQUENCY_IN_HZ = 1000
SAMPLE_SIZE_IN_BITS = 16
FORMAT = I2S.MONO  # this example suports only MONO
SAMPLE_RATE_IN_HZ = 22_050


def make_tone(rate, bits, frequency, volume):
    # create a buffer containing the pure tone samples
    samples_per_cycle = rate // frequency
    sample_size_in_bytes = bits // 8
    samples = bytearray(samples_per_cycle * sample_size_in_bytes)
    volume_reduction_factor = 65536 // volume   # value reducing the volume = 2^16
    range = pow(2, bits) // 2 // volume_reduction_factor
    
    if bits == 16:
        format = "<h"
    else:  # assume 32 bits
        format = "<l"
    
    for i in range(samples_per_cycle):
        sample = range + int((range - 1) * math.sin(2 * math.pi * i / samples_per_cycle))
        struct.pack_into(format, samples, i * sample_size_in_bytes, sample)
        
    return samples


audio_out = I2S(
    I2S_ID,
    sck = Pin(AMP_SCK_PIN),
    ws = Pin(AMP_WS_PIN),
    sd = Pin(AMP_SD_PIN),
    mode = I2S.TX,
    bits = SAMPLE_SIZE_IN_BITS,
    format = FORMAT,
    rate = SAMPLE_RATE_IN_HZ,
    ibuf = BUFFER_LENGTH_IN_BYTES,
)


volume = 32  # number from 0 to 4096
samples = make_tone(SAMPLE_RATE_IN_HZ, SAMPLE_SIZE_IN_BITS, TONE_FREQUENCY_IN_HZ, volume)

# global variables used by second thread
distance = 30.0
enable_distance_meter = False


def second_thread_func():
    global distance
    global enable_distance_meter
    global exit_thread_sig
    print("Starting second thread")
    try:
        while True:
            if enable_distance_meter:
                distance = distance_meter.distance_cm()
            
                if distance > 22:
                    sleep_ms(50)
            
                elif distance <= 22 and distance > 16:
                    for i in range (1,200):
                        num_written = audio_out.write(samples)
                    sleep_ms(800)
            
                elif distance <= 16 and distance > 11:
                    for i in range (1,190):
                        num_written = audio_out.write(samples)
                    sleep_ms(420)
                
                elif distance <= 11 and distance > 7:
                    for i in range (1,150):
                        num_written = audio_out.write(samples)
                    sleep_ms(250)
            
                elif distance <= 7 and distance > 3.5:
                    for i in range (1,100):
                        num_written = audio_out.write(samples)
                    sleep_ms(80)

                elif distance <= 3.5:
                    for i in range (1,800):
                        num_written = audio_out.write(samples)
                elif exit_thread_sig:
                    _thread.exit()
                
            elif exit_thread_sig:
                _thread.exit()
            else:
                sleep_ms(50)
    
    except (KeyboardInterrupt, Exception) as e:
        print("caught exception {} {}".format(type(e).__name__, e))
        if exit_thread_sig:
            _thread.exit()



#-------------------------------------------------------------------
##########################   MAIN LOOP   ###########################
#-------------------------------------------------------------------


# activating a WLAN interface
w0 = network.WLAN(network.STA_IF)
w0.active(True)
w0.config(mac = b'\xaa\xaa\xaa\xaa\xaa\xaa')  # setting MAC adress of local device

e = espnow.ESPNow()
e.init()
peer = b'\xaa\xaa\xaa\xaa\xaa\xab'  # MAC address of peer's wifi interface
e.add_peer(peer)

print("Starting program")

exit_thread_sig = False

_thread.start_new_thread(second_thread_func, ())


try:
    while True:
        host, msg = e.irecv()
        
        if msg:             # msg == None if timeout in irecv()
            print (msg)
            
            if msg == b'a':
                ledR_g.value(not ledR_g.value())

            elif msg == b'b':
                ledG_g.value(not ledG_g.value())

            elif msg == b'c':
                ledB_g.value(not ledB_g.value())

            elif msg == b'd':
                ledR_g.off()
                ledG_g.off()
                ledB_g.off()

            elif msg == b'e':
                enable_distance_meter = True
            
            elif msg == b'f':
                enable_distance_meter = False
            
            if msg == b'g':
                ledR_o.value(not ledR_o.value())

            elif msg == b'h':
                ledG_o.value(not ledG_o.value())

            elif msg == b'i':
                ledB_o.value(not ledB_o.value())

            elif msg == b'j':
                ledR_o.off()
                ledG_o.off()
                ledB_o.off()
            
            if msg == b'k':
                ledR_b.value(not ledR_b.value())

            elif msg == b'l':
                ledG_b.value(not ledG_b.value())

            elif msg == b'm':
                ledB_b.value(not ledB_b.value())

            elif msg == b'n':
                ledR_b.off()
                ledG_b.off()
                ledB_b.off()
                
            elif msg == b'1':
                s1.angle(635) # opening garage gate

            elif msg == b'2':
                s1.angle(635,-1) # closing garage gate

            elif msg == b'end':
                print("End signal catched, ending program.")
                break
                
                
except (KeyboardInterrupt, Exception) as e:
    print("caught exception {} {}".format(type(e).__name__, e))
    ledR_g.off()
    ledG_g.off()
    ledB_g.off()
    ledR_o.off()
    ledG_o.off()
    ledB_o.off()
    ledR_b.off()
    ledG_b.off()
    ledB_b.off()

# cleanup
exit_thread_sig = True
sleep_ms(50)
audio_out.deinit()
print("Done")
_thread.exit()


