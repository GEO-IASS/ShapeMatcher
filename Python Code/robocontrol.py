import serial
import time

count1 =0
count2 =0
count3 =0
count4 =0

port = serial.Serial('COM4', baudrate=9600, timeout=None)

def pickup():
    servo(2, 70)

    servo(1, 170)
    time.sleep(0.25)
    servo(2, 175)
    time.sleep(0.5)
    servo(1, 90)
    time.sleep(0.4)
    # time.sleep(1) #no feedback from servo complete task so need to wait before writing another angle.
    # servo(1,125)
    servo(2, 70)
    return

def drop():
    servo(2, 175)
    time.sleep(0.5)
    servo(1, 180)
    time.sleep(0.5)
    servo(2, 70)
    time.sleep(0.4)
    # time.sleep(1) #no feedback from servo complete task so need to wait before writing another angle.
    # servo(1,125)
    return


def stop():
    port.write('\x10')
    port.write('\x00')
    port.write('\x00')
    port.write('\x00')
    print("Bot has stopped movement.")
    return

def forward_real():
    global count1
    count1 = count1+1
    forward(184)
    time.sleep(0.2)
    forward_correct()
    return


def about_turn():
    #about turn------------------------
    forward(30)
    time.sleep(0.25)
    left(179)
    time.sleep(0.25)
    backward(40)
    return

def right_real():
    global count3
    count3 = count3 +1
    forward(36)
    time.sleep(0.25)
    right(86)
    time.sleep(0.25)
    backward(42)
    right_correct()
    return

def left_real():
    global count4
    count4 = count4+1
    forward(36)
    time.sleep(0.25)
    left(86)
    time.sleep(0.25)
    backward(42)
    left_real()
    return

def forward(distance=None):
    #max distance = 2040mm
    port.write('\x11')
    if distance is not None:
        d = distance/8
        t = chr(d)
        port.write(t)
        port.write('\x01')
        port.write('\x00')
        #print("Waiting for task completion...")
        port.read(1)
        #print("Forward task complete.")
        port.flushInput()
    if distance is None:
        port.write('\x00')
        port.write('\x00')
        port.write('\x00')
        #print("Bot is in forward motion.")
    return

def backward(distance=None):
    global count2
    count2 = count2 +1
    #max distance = 2040mm
    port.write('\x12')
    if distance is not None:
        d = distance/8
        t = chr(d)
        port.write(t)
        port.write('\x01')
        port.write('\x00')
        #print("Waiting for task completion...")
        port.read(1)
        #print("Backward task complete.")
        port.flushInput()
    if distance is None:
        port.write('\x00')
        port.write('\x00')
        port.write('\x00')
        #print("Bot is in backward motion.")
    return

#normal right (2 wheels, left forward, right backward)
def right(degrees):
    #degrees = 0-360 but only multiples of 2
    half = degrees/2
    d = chr(half)
    port.write('\x13')
    port.write(d)
    port.write('\x01')
    port.write('\x00')
    #print("Waiting for task completion...")
    port.read(1)
    #print("Rotation task complete.")
    port.flushInput()
    return

#soft right 1(1 wheel, left wheel forward, right wheel stationary)
def right1(degrees):
    #degrees = 0-360 but only multiples of 2
    half = degrees/2
    d = chr(half)
    port.write('\x1B')
    port.write(d)
    port.write('\x01')
    port.write('\x00')
    #print("Waiting for task completion...")
    port.read(1)
    #print("Rotation task complete.")
    port.flushInput()
    return

#soft right 2(1 wheel, left wheel stationary, right wheel backward)
def right2(degrees):
    #degrees = 0-360 but only multiples of 2
    half = degrees/2
    d = chr(half)
    port.write('\x1D')
    port.write(d)
    port.write('\x01')
    port.write('\x00')
    #print("Waiting for task completion...")
    port.read(1)
    #print("Rotation task complete.")
    port.flushInput()
    return

#normal left (2 wheels, left backward, right forward)
def left(degrees):
    #degrees = 0-360 but only multiples of 2
    half = degrees/2
    d = chr(half)
    port.write('\x14')
    port.write(d)
    port.write('\x01')
    port.write('\x00')
    #print("Waiting for task completion...")
    port.read(1)
    #print("Rotation task complete.")
    port.flushInput()
    return

#soft left 1(1 wheel,left wheel stationary,right wheel forward)
def left1(degrees):
    #degrees = 0-360 but only multiples of 2
    half = degrees/2
    d = chr(half)
    port.write('\x1C')
    port.write(d)
    port.write('\x01')
    port.write('\x00')
    #print("Waiting for task completion...")
    port.read(1)
    #print("Rotation task complete.")
    port.flushInput()
    return

#soft left 2(1 wheel,left wheel backward,right wheel stationary)
def left2(degrees):
    #degrees = 0-360 but only multiples of 2
    half = degrees/2
    d = chr(half)
    port.write('\x1E')
    port.write(d)
    port.write('\x01')
    port.write('\x00')
    #print("Waiting for task completion...")
    port.read(1)
    #print("Rotation task complete.")
    port.flushInput()
    return

def frontsensor(number):
    port.write('\x15')
    n = chr(number)
    port.write(n)
    port.write('\x01')
    port.write('\x00')
    val = port.read(1)
    value = '%d' %int(ord(val))
    port.flushInput()
    return value

def sharpdistance():
    port.write('\x16')
    port.write('\x11')
    port.write('\x01')
    port.write('\x00')
    val = port.read(1)
    distance = int(ord(val))
    print "Raw Value: " , distance
    converted = ((distance*800)/255)
    print "Distance: ", converted,"mm"
    port.flushInput()
    return converted

def servo(number,degrees):
    #number = servo number (1,2)
    #degrees = 0-180
    port.write('\x17')
    n = chr(number)
    port.write(n)
    d = chr(degrees)
    port.write(d)
    port.write('\x00')
    port.read(1)
    port.flushInput()
    return

def velocity(left,right):
    #left = 0-255
    #right = 0-255
    port.write('\x18')
    l = chr(left)
    r = chr(right)
    port.write(l)
    port.write(r)
    port.write('\x00')
    port.read(1)
    print("Velocity is set.")
    port.flushInput()
    return

def buzzer(switch):
    port.write('\x19')
    s = chr(switch)
    port.write(s)
    port.write('\x00')
    port.write('\x00')
    port.read(1)
    if (switch is 1):
        print("Buzzer turned on")
    elif (switch is 0):
        print("Buzzer turned off")
    port.flushInput()
    return

def backsensor(channel):
    port.write('\x1A')
    ch = chr(channel)
    port.write(ch)
    port.write('\x01')
    port.write('\x00')
    val = port.read(1)
    value = '%d' %int(ord(val))
    port.flushInput()
    return value

def turnoffsensors():
    port.write('\x1F')
    port.write('\x00')
    port.write('\x01')
    port.write('\x00')
    port.read(1)
    print("Sensors turned off.")
    port.flushInput()
    return

def turnonsensors():
    port.write('\x20')
    port.write('\x00')
    port.write('\x01')
    port.write('\x00')
    port.read(1)
    print("Sensors turned on.")
    port.flushInput()
    return

def readVoltage():
    port.flushInput()
    port.write('\x21')
    port.write('\x00')
    port.write('\x01')
    port.write('\x00')
    val = port.read(1)
    value = int(ord(val))
    print "Raw Value: " , value
    converted = ((value*1027)/255)
    print "Corrected Reading: ", converted
    port.flushInput()
    return

turnoffsensors()
velocity(254, 250)

def left_correct():
    global count4
    if(count4 > 4):
        left(6)
        count4 =0
    return

def right_correct():
    global count3
    if(count3 >3):
        right(6)
        count3 =0
    return

def forward_correct():
    global count1
    if(count1 > 6):
        forward(10)
        count1 = 0
    return
