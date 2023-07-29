import  time, pyb , sensor, image, lcd, os
from pyb import  Pin, LED
sensor.reset() # Initialize the camera sensor.
sensor.set_pixformat(sensor.GRAYSCALE) # or sensor.GRAYSCALE
sensor.set_framesize(sensor.QQVGA) # Special 128x160 framesize for LCD Shield.
lcd.init() # Initialize the lcd screen.
pin0 = Pin('P4', Pin.IN, Pin.PULL_UP)
pin1 = Pin('P5', Pin.OUT_PP, Pin.PULL_NONE) #external LEDs

newName='I_' #prefix of file name
counterNumber=1 #numbher of file start from...
countCheck = 0

RED_LED_PIN = 1
GREEN_LED_PIN = 2
BLUE_LED_PIN = 3
IR_LED_PIN =4

# Make a temp directory
if not "images" in os.listdir(): os.mkdir("images")

##internal LEDS
#pyb.LED(RED_LED_PIN).off()
#pyb.LED(GREEN_LED_PIN).off()
#pyb.LED(BLUE_LED_PIN).off()
#pyb.LED(IR_LED_PIN).off()
##LOOP
#while(True):
#    lcd.display(sensor.snapshot()) # Take a picture and display the image.
#    for i in range(4):
#        if pin0.value()==0:
#            countCheck += 1
#            time.sleep_ms(50)
#        else:
#            countCheck = 0
#    if countCheck>=3:
#          countCheck = 0
#          pyb.LED(RED_LED_PIN).on()
#          pin1.value(1)
#          img = sensor.snapshot() # make foto
#          #img.save("example.jpg")
#          img.save('images/' + newName+counterNumber, quality=90)
#          counterNumber +=1
#          countCheck +=1
#          time.sleep_ms(1000)
#          pyb.LED(RED_LED_PIN).off()
#          pin1.value(0)
