from barcode import barcode_reader
barcode= barcode_reader()
print(barcode)
import RPi.GPIO as GPIO
import time
GPIO.cleanup()
#GPIO Mode (BOARD / BCM)
GPIO.setmode(GPIO.BCM)
def oxy():
    from heartrate_monitor import HeartRateMonitor
    import time
    import argparse
    from barcode import barcode_reader
    parser = argparse.ArgumentParser(description="Read and print data from MAX30102")
    parser.add_argument("-r", "--raw", action="store_true",
                    help="print raw data instead of calculation result")
    parser.add_argument("-t", "--time", type=int, default=10,
                    help="duration in seconds to read from sensor, default 30")
    args = parser.parse_args()
    print('sensor starting...')
    hrm = HeartRateMonitor(print_raw=args.raw, print_result=(not args.raw))
    a = hrm.start_sensor()
    try:
        time.sleep(args.time)
    except KeyboardInterrupt:
        print('keyboard interrupt detected, exiting...')
    hrm.stop_sensor()
    print('sensor stoped!')
    o = a[-10:]
    final = max(o)
    return final

def temp():
    import Adafruit_GPIO.I2C as I2C
    I2C.require_repeated_start()
    class Melexis:
        def __init__(self, address=0x5A):
            self._i2c = I2C.Device(address,busnum=1)
        def readAmbient(self):
            return self._readTemp(0x06)
        def readObject1(self):
            return self._readTemp(0x07)
        def readObject2(self):
            return self._readTemp(0x08)
        def _readTemp(self, reg):
            temp = self._i2c.readS16(reg)
            temp = temp * .02 - 273.15
            return temp
    #if name == "main":
    sensor = Melexis()
    t = sensor.readObject1()
    t1 = (t * 1.8) + 32
    return t1

def ultra1():
    import RPi.GPIO as GPIO
    import time

    #GPIO Mode (BOARD / BCM)
    GPIO.setmode(GPIO.BCM)

    #set GPIO Pins
    GPIO_TRIGGER = 17
    GPIO_ECHO = 27

    #set GPIO direction (IN / OUT)
    GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
    GPIO.setup(13, GPIO.OUT)
    GPIO.setup(GPIO_ECHO, GPIO.IN)

    def distance1():
        time.sleep(1)
        GPIO.setup(13, GPIO.OUT)
        # set Trigger to HIGH
        GPIO.output(GPIO_TRIGGER, True)

        # set Trigger after 0.01ms to LOW
        time.sleep(0.00001)
        GPIO.output(GPIO_TRIGGER, False)

        StartTime = time.time()
        StopTime = time.time()

        # save StartTime
        while GPIO.input(GPIO_ECHO) == 0:
            StartTime = time.time()

        # save time of arrival
        while GPIO.input(GPIO_ECHO) == 1:
            StopTime = time.time()

        # time difference between start and arrival
        TimeElapsed = StopTime - StartTime
        # multiply with the sonic speed (34300 cm/s)
        # and divide by 2, because there and back
        distance = (TimeElapsed * 34300) / 2
        # GPIO.cleanup()
        return distance
    # print(distance())
    # GPIO.cleanup()
    while True:
        distance = distance1()
        if distance > 2500:
            distance = 2000
        print(distance)
        if distance>=2 and distance<=8:
            print("Measuring")
            GPIO.output(13,GPIO.HIGH)
            t = temp()
            return t


        if distance>7:
            GPIO.output(13,GPIO.LOW)






def ultra():
    import RPi.GPIO as GPIO
    import time
    GPIO.setmode(GPIO.BCM)
    TRIG = 10
    ECHO = 9
    GPIO.setup(TRIG,GPIO.OUT)
    GPIO.setup(ECHO,GPIO.IN)
    GPIO.setup(11,GPIO.OUT)

    GPIO.output(TRIG, False)
    print ("Calibrating.....")
    time.sleep(2)

    print ("Place the object......")



    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)

    while GPIO.input(ECHO)==0:
      pulse_start = time.time()

    while GPIO.input(ECHO)==1:
      pulse_end = time.time()

    pulse_duration = pulse_end - pulse_start

    distance = pulse_duration * 17150

    distance = round(distance+1.15, 2)
    print(distance)
    if distance<=15 and distance>=5:
        GPIO.output(19,GPIO.HIGH)
        time.sleep(5)
        GPIO.output(19,GPIO.LOW)


    if distance>15:
        print ("COME CLOSER....")






    import RPi.GPIO as GPIO
    import time

    #GPIO Mode (BOARD / BCM)
    GPIO.setmode(GPIO.BCM)

    #set GPIO Pins
    GPIO_TRIGGER = 10
    GPIO_ECHO = 9

    #set GPIO direction (IN / OUT)
    GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
    GPIO.setup(19, GPIO.OUT)
    GPIO.setup(GPIO_ECHO, GPIO.IN)

    def distance1():
        GPIO.setup(19, GPIO.OUT)
        # set Trigger to HIGH
        GPIO.output(GPIO_TRIGGER, True)

        # set Trigger after 0.01ms to LOW
        time.sleep(0.00001)
        GPIO.output(GPIO_TRIGGER, False)

        StartTime = time.time()
        StopTime = time.time()

        # save StartTime
        while GPIO.input(GPIO_ECHO) == 0:
            StartTime = time.time()

        # save time of arrival
        while GPIO.input(GPIO_ECHO) == 1:
            StopTime = time.time()

        # time difference between start and arrival
        TimeElapsed = StopTime - StartTime
        # multiply with the sonic speed (34300 cm/s)
        # and divide by 2, because there and back
        distance = (TimeElapsed * 34300) / 2
        # GPIO.cleanup()
        return distance
    # print(distance())
    # GPIO.cleanup()
    while True:
        distance = distance1()

        print(distance)
        if distance<=15 and distance>=5:
            GPIO.output(19,GPIO.HIGH)
            time.sleep(5)
            GPIO.output(19,GPIO.LOW)


        else:
            GPIO.output(19,GPIO.LOW)


















# def display():
import RPi.GPIO as GPIO
import time


LCD_RS = 7
LCD_E  = 8
LCD_D4 = 25
LCD_D5 = 24
LCD_D6 = 23
LCD_D7 = 18


LCD_WIDTH = 16    # Maximum characters per line
LCD_CHR = True
LCD_CMD = False

LCD_LINE_1 = 0x80 # LCD RAM address for the 1st line
LCD_LINE_2 = 0xC0 # LCD RAM address for the 2nd line

# Timing constants
E_PULSE = 0.0005
E_DELAY = 0.0005

def main(temperature, oxygen):
  # Main program block
  GPIO.setwarnings(False)
  GPIO.setmode(GPIO.BCM)       # Use BCM GPIO numbers
  GPIO.setup(LCD_E, GPIO.OUT)  # E
  GPIO.setup(LCD_RS, GPIO.OUT) # RS
  GPIO.setup(LCD_D4, GPIO.OUT) # DB4
  GPIO.setup(LCD_D5, GPIO.OUT) # DB5
  GPIO.setup(LCD_D6, GPIO.OUT) # DB6
  GPIO.setup(LCD_D7, GPIO.OUT) # DB7
  lcd_init()
  ox = "oxy = " + str(oxygen)
  tem = "temp = " + str(temperature)
  lcd_string(ox,LCD_LINE_1)
  lcd_string(tem,LCD_LINE_2)
  time.sleep(3)

def lcd_init():
  # Initialise display
  lcd_byte(0x33,LCD_CMD) # 110011 Initialise
  lcd_byte(0x32,LCD_CMD) # 110010 Initialise
  lcd_byte(0x06,LCD_CMD) # 000110 Cursor move direction
  lcd_byte(0x0C,LCD_CMD) # 001100 Display On,Cursor Off, Blink Off
  lcd_byte(0x28,LCD_CMD) # 101000 Data length, number of lines, font size
  lcd_byte(0x01,LCD_CMD) # 000001 Clear display
  time.sleep(E_DELAY)

def lcd_byte(bits, mode):
  # Send byte to data pins
  # bits = data
  # mode = True  for character
  #        False for command

  GPIO.output(LCD_RS, mode) # RS

  # High bits
  GPIO.output(LCD_D4, False)
  GPIO.output(LCD_D5, False)
  GPIO.output(LCD_D6, False)
  GPIO.output(LCD_D7, False)
  if bits&0x10==0x10:
    GPIO.output(LCD_D4, True)
  if bits&0x20==0x20:
    GPIO.output(LCD_D5, True)
  if bits&0x40==0x40:
    GPIO.output(LCD_D6, True)
  if bits&0x80==0x80:
    GPIO.output(LCD_D7, True)

  # Toggle 'Enable' pin
  lcd_toggle_enable()

  # Low bits
  GPIO.output(LCD_D4, False)
  GPIO.output(LCD_D5, False)
  GPIO.output(LCD_D6, False)
  GPIO.output(LCD_D7, False)
  if bits&0x01==0x01:
    GPIO.output(LCD_D4, True)
  if bits&0x02==0x02:
    GPIO.output(LCD_D5, True)
  if bits&0x04==0x04:
    GPIO.output(LCD_D6, True)
  if bits&0x08==0x08:
    GPIO.output(LCD_D7, True)

  # Toggle 'Enable' pin
  lcd_toggle_enable()

def lcd_toggle_enable():
  # Toggle enable
  time.sleep(E_DELAY)
  GPIO.output(LCD_E, True)
  time.sleep(E_PULSE)
  GPIO.output(LCD_E, False)
  time.sleep(E_DELAY)

def lcd_string(message,line):
  # Send string to display

  message = message.ljust(LCD_WIDTH," ")

  lcd_byte(line, LCD_CMD)

  for i in range(LCD_WIDTH):
    lcd_byte(ord(message[i]),LCD_CHR)


main(00,00)
oxygen = oxy()
print(oxygen)
temperature = ultra1()
main(temperature, oxygen)

#
import firebase_admin
import google.cloud
from firebase_admin import credentials ,firestore
cred = credentials.Certificate("/home/pi/Desktop/Codes/Sanitizer/final/entracer-d6a76-firebase-adminsdk-mon4o-b5af82e64a.json")
app = firebase_admin.initialize_app(cred)
def init():

    db = firestore.client()
    doc_ref = db.collection(u'roll')
    return doc_ref
doc_ref=init()
ex = int(barcode)
data={u"temp":temperature,
          u"oxy":oxygen,
         u"id":ex,
           }
doc_ref.document(barcode).set(data)

# ultra()

GPIO.cleanup()
