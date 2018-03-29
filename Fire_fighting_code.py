#Fire Fighting Robot Code
#Isaiah Frey and Jacob Stratman
#This robot will navigate a maze and extinguish a fire.
#!/usr/bin/env python3

"""-------------------------------------Update Log----------------------------------
Version
Major Update.Medium Update. Minor Update/Bug fix
V 1.0 - Initial programming
V 2.0 - Alpha testing
V 3.0 - Beta testing

V 1.0.0    12/1/17
   Added 4 pwm ports to move motors forward and backward
V 1.1.0    12/2/17
   Changed to 2 pwm pins and a directional variable
V 1.2.0    1/8/18
   Fixed encoder code and added Move_forward_distance Function
V 1.3.0    1/22/18
   Added ADC setup code to main program and Find_short_dist Function
V 1.3.1    1/23/18
   Fixed incompatibility between ADC setup and main setup code
   Switched to BCM mode for consistency  
V 1.4.0    1/23/18
   Programmed read short distance sensor function and turn until aligned
   Added main loop and test loop
V 1.5.0    1/24/18
   Programmed right wall following
   Programmed read long distance sensor
V 1.5.1    1/26/18
   Made program run upon boot
V 1.5.2    1/29/18
   Programmed test loop to test all movement functions
V 1.6.0    2/2/18
   Programmed room measuring and room determination
V 1.6.1    2/4/18
   Fixed Turn_until_aligned bounding bug
V 1.6.2    2/5/18
   Improved wall following code
V 2.0.0    2/8/18
   Robot now wall follows.
V 2.1.0    2/12/18
   Robot exit first room code tested and works successfully
   This includes Turn_until_aligned, Move_forward, and Turn_left
V 2.1.1    2/14/18
   Edited wall following to allow for right turns as well as left turns
V 2.1.2    2/16/18
   Began construction of beginning sequence in main loop
V 2.1.3    2/19/18
   Added Determine_entrance function to allow the robot to determine which door it is at
V 2.2.0    2/20/18
   Added programming mode to keep the robot from running during programming if right back sensor is blocked
   Added config variables to allow the robot to remember what configuration the maze is in.
   Set a general plan for how to navigate the maze in the main loop
V 2.3.0    2/21/18
   Added line sensors and line sensor function Check_line
   Robot now checks for a line during a right turn approximately once per millimeter
V 2.4.0    2/23/18
   Fixed edge cases for navigating out of first room
   Added Search_for_flame function and double check feature
V 2.4.1    2/24/18
   Added instructions for navigation from position 1, 2, and 3
   Changed Move_forward_while_checking function to also check for a line
   Added checking for a dog to Right_wall_follow_until_door function
V 2.4.2    2/25/18
   Fixed multiple variable passing errors with Navigate_out_of_room function
   Adjusted area calculation in Determine_room to be width*length instead of adding up sensors
   Added an additional 2 cm buffer for line sensors on length of Determine_room
V 2.4.3    2/26/18
   Edited and tested Measure_room function
   Robot successfully  moves from room 1 to room 2 and room 2 to room 3
V 3.0.0    2/27/18
   Finished all basic positions for naviagation
   Created a Align_to_line function to line up the bot whenever it passes over a door
V 3.0.1    2/28/18
   Edited wall following function, no longer delays for .02 seconds between samples
   Added line checking between each sensor reading during wall following
   Made line checking 3X as frequent during right turns during wall following
   Added room checked array to help determine which rooms the robot has already been in
V 3.0.2    3/1/18
   Edited Search_room function to be room specific
V 3.0.3    3/1/18
   Edited Search _room to apply to all rooms and always search for a left wall
   Edited position 3 code to more accurately align with hall 4
   Edited position 2 code to include the entrance position 2 ends at in room 3
V 3.1.0    3/1/18
   Edited position 6 code to move the robot from the left wall of room 4 to the right wall of room 1
   for greater accuracy
   Robot completes  navigation of the maze of the first time
V 3.1.1    3/12/18
   Adjusted room searching code for room 3
V 3.1.2    3/14/18
   Fixed entrance code in room 3
   Adjusted  position 5 to work if robot starts in room 4
V 3.1.3    3/16/18
   Programmed and tested UV Tron
   Programmed and created a test for MOSFET for fire extinguisher
V 3.2.0    3/17/18
   Programmed Detect_flame and Extinguish_flame functions
   Improved Search_room function to actually search for a fire
   Fixed error in Align_to_line function where bot would continually align if it missed the line
V 3.2.1    3/17/18
   Improved Extinguish_flame function to turn toward the fire and approach it between scans
V 3.2.2    3/18/18
   Improved Search_room function to return robot to previous position after searching for fire
   Improved Extinguish_flame function to approach proportional to the square root of the distance
V 3.3.0    3/24/18
   Added Sound_start functionwhich detects a 3.5-4.1 kHz signal and starts the robot
"""

#----------------------------------------Assignments---------------------------------#
#Wheel diameter approximately 10cm
#Room areas 1: 6264 cm^2
#           2: 7416 cm^2
#           3: 11590 cm^2
#           4: 3570 cm^2

#Motor Assignments
#GPIO 17 - Left motor
#GPIO 27 - Right motor
#GPIO 22 - Left motor direction
#GPIO 10 - Right motor direction

#Encoder Assignments
#GPIO 23 - Left encoders
#GPIO 5 - Right encoders

#8 Channel ADC
#Channel 0 - Front Left Sensor
#Channel 1 - Front Right Sensor
#Channel 2 - Right Front Sensor
#Channel 3 - Front Middle Sensor
#Channel 4 - Long Range Left Sensor
#Channel 5 - Long Range Back Sensor
#Channel 6 - Phototransister
#Channel 7 -

#Line sensors
#GPIO 4 - Left
#GPIO 14 - Middle
#GPIO 15 - Right

#Servo Motor
#GPIO 21 - Servo PWM

#UV Tron
#GPIO 12 - UV Tron (Default high, pulses when detects fire)

#Extinguisher switch
#GPIO 16 - MOSFET

#LEDs
#GPIO 6 - Sound LED
#GPIO 13 - Fire LED

#-------------------------------------Import Libraries--------------------------------#
import RPi.GPIO as GPIO           #Import a library to setup GPIO
import time                       #Import a library to setup clocks
from time import sleep            #Import the sleep function for delays
import Adafruit_GPIO.SPI as SPI   #Import SPI (Serial Peripherial Interface)
import Adafruit_MCP3008           #Import a library for ADC
from math import atan             #Import the arc tangent function from math
from math import sqrt             #Import the square root funtion from math
import pyaudio                    #Import a library for sound detection
from numpy import zeros,linspace,short,fromstring,hstack,transpose,log  #import functions for  signal analysis
from scipy import fft             #Import a function to perform the fast fourier transform

#--------------------------------------Initializations--------------------------------#
GPIO.setmode(GPIO.BCM)      #Set the pins to the BOARD configuration
GPIO.setwarnings(False)     #Do not warn the user about pin initialization

GPIO.setup(17, GPIO.OUT)    #Set GPIO 17 as output
GPIO.setup(27, GPIO.OUT)    #Set GPIO 27 as output
GPIO.setup(22, GPIO.OUT)    #Set GPIO 22 as output for left motor direction
GPIO.setup(10, GPIO.OUT)    #Set GPIO 10 as output for right motor direction

GPIO.setup(4, GPIO.IN,)     #Set GPIO 4 as the input for the line sensors
GPIO.setup(14, GPIO.IN,)    #Set GPIO 14 as the input for the line sensors
GPIO.setup(15, GPIO.IN,)    #Set GPIO 15 as the input for the line sensors

GPIO.setup(23, GPIO.IN)     #Set GPIO 23 to input for encoders
GPIO.setup(5, GPIO.IN)      #Set GPIO 5 to input for encoders

GPIO.setup(21, GPIO.OUT)    #Set GPIO 21 as output for servo

GPIO.setup(12, GPIO.IN)     #Set GPIO 12 as input for UV Tron

GPIO.setup(16, GPIO.OUT)    #Set GPIO 16 as output for extinguisher switch

GPIO.setup(6, GPIO.OUT)     #Set GPIO 6 as output for fire detect LED
GPIO.setup(13, GPIO.OUT)    #Set GPIO 13 as output for sound detect LED

pwm_left = GPIO.PWM(17,100) #Initialize GPIO 17 at a 100 Hz frequency
pwm_right = GPIO.PWM(27,100)#Initialize GPIO 27 at a 100 Hz frequency
pwm_servo = GPIO.PWM(21,90)#Initialize GPIO 21 at a 100 Hz frequency

###TEST
GPIO.setup(16, GPIO.OUT)

# Setup software SPI configuration:
CLK  = 18
MISO = 23
MOSI = 24
CS   = 25
mcp = Adafruit_MCP3008.MCP3008(clk=CLK, cs=CS, miso=MISO, mosi=MOSI)

#Variable Initializations
start = False         #Set this true when a 3.8 kHz signal is detected
fire_extinguished = False  #Set to true when the fire has been put out
level = 1             #Set this for the level of the maze the robot is running
room3_config = 0      #0 for unknown, 1 for lower entrance, 2 for upper entrance
room4_config = 0      #0 for unknown, 1 for lower entrance, 2 for upper entrance
dog_config = 0        #0 for unknown, 1 for lower placement, 2 for right, 3 for upper
position = 0          #Set to a number 1-5 for based on the part of the sequence the robot is at
hall = 0              #Set to a number 1-6 based on which hallway the robot is in
room = 0              #Initialize the room to 0 (none of the rooms)
inside_room = True    #Set to false when in a hall and true when in a room
line_detected = False #Set to true when a line is detected
room_checked = [False,False,False,False] #Set an array to mark off which rooms were checked
outside_room4 = False #Set a variable to tell the robot if it has just determined the entrance of room 4

#----------------------------------------Definitions-----------------------------------#
#Speeds
TURNSPEED = 13.6  #Speed robot moves while turning
SPEED = 35        #Speed robot moves while going forward (optimal at 35)

#Delays
TURN = .02        #Time delay the motors must run to turn 1 degree
CONVCM = .2       #Convert the time delay to centimeters
PAUSE = .2        #Wait time between movements
TRANS = .5        #Set the transition delay
EXTINGUISH = 1    #Set the time the Versa Valve will be open

#Constants
PI = 3.1415926535 #Define pi

#Tolerances
ALIGNTOL = .5     #Tolerance in cm that front sensors must be before driving forward
OBSTACLETOL = 5   #Tolerance in cm that the side sensors must differe from the center sensor to be an obstacle
MAXSHORT = 25     #Tolerance for how far in cm short range distance sensor can pick up
WALLDIST = 12     #Tolerance between robot and wall while wall folowing
FRONTDIST = 12    #Tolerance between robot and obstacles in front

#Dimensions
WIDTH = 20        #Define the robot width in cm
LENGTH = 23       #Define the robot length in cm

#PD Controller
P = 6             #Define proportional constant
D = .12           #Define derivitave constant
T = .02           #Define time constant

#-----------------------------------------FUNCTIONS------------------------------------#
#--------------------------------------Sound Detection---------------------------------#

def Sound_start():
    double_check = 0
    #Volume Sensitivity, 0.05: Extremely Sensitive, may give false alarms
    #             0.1: Probably Ideal volume
    #             1: Poorly sensitive, will only go off for relatively loud
    SENSITIVITY= 1.0
    # Alarm frequencies (Hz) to detect (Use audacity to record a wave and then do Analyze->Plot Spectrum)
    TONE = 3500
    #Bandwidth for detection (i.e., detect frequencies within this margin of error of the TONE)
    BANDWIDTH = 30
    #How many 46ms blips before we declare a beep? (Take the beep length in ms, divide by 46ms, subtract a bit)
    beeplength=8
    # How many beeps before we declare an alarm?
    alarmlength=5
    # How many false 46ms blips before we declare the alarm is not ringing
    resetlength=10
    # How many reset counts until we clear an active alarm?
    clearlength=30
    # Enable blip, beep, and reset debug output
    debug=False
    # Show the most intense frequency detected (useful for configuration)
    frequencyoutput=True


    #Set up audio sampler - 
    NUM_SAMPLES = 2048
    SAMPLING_RATE = 44100
    pa = pyaudio.PyAudio()
    _stream = pa.open(format=pyaudio.paInt16,
                      channels=1, rate=SAMPLING_RATE,
                      input=True,
                      frames_per_buffer=NUM_SAMPLES)

    print("Alarm detector working. Press CTRL-C to quit.")

    blipcount=0
    beepcount=0
    resetcount=0
    clearcount=0
    alarm=False
    thefreq = 0
    
    while (double_check < 2):
        while (thefreq < 3420 or thefreq > 4180):
            while _stream.get_read_available()< NUM_SAMPLES: sleep(0.01)
            audio_data  = fromstring(_stream.read(
                 _stream.get_read_available(), exception_on_overflow = False), dtype=short)[-NUM_SAMPLES:]
            # Each data point is a signed 16 bit number, so we can normalize by dividing 32*1024
            normalized_data = audio_data / 32768.0
            intensity = abs(fft(normalized_data))[:int(NUM_SAMPLES/2)]
            frequencies = linspace(0.0, float(SAMPLING_RATE)/2, num=NUM_SAMPLES/2)
            if frequencyoutput:
                which = intensity[1:].argmax()+1
                # use quadratic interpolation around the max
                if which != len(intensity)-1:
                    y0,y1,y2 = log(intensity[which-1:which+2:])
                    x1 = (y2 - y0) * .5 / (2 * y1 - y2 - y0)
                    # find the frequency and output it
                    thefreq = (which+x1)*SAMPLING_RATE/NUM_SAMPLES
                else:
                    thefreq = which*SAMPLING_RATE/NUM_SAMPLES
                print("\t\t\t\tfreq=",thefreq)
            if max(intensity[(frequencies < TONE+BANDWIDTH) & (frequencies > TONE-BANDWIDTH )]) > max(intensity[(frequencies < TONE-1000) & (frequencies > TONE-2000)]) + SENSITIVITY:
                blipcount+=1
                resetcount=0
                if debug: print("\t\tBlip",blipcount)
                if (blipcount>=beeplength):
                    blipcount=0
                    resetcount=0
                    beepcount+=1
                    if debug: print("\tBeep",beepcount)
                    if (beepcount>=alarmlength):
                        clearcount=0
                        alarm=True
                        print("Alarm!")
                        beepcount=0
            else:
                blipcount=0
                resetcount+=1
                if debug: print("\t\t\treset",resetcount)
                if (resetcount>=resetlength):
                    resetcount=0
                    beepcount=0
                    if alarm:
                        clearcount+=1
                        if debug: print("\t\tclear",clearcount)
                        if clearcount>=clearlength:
                            clearcount=0
                            print("Cleared alarm!")
                            alarm=False
            if (thefreq < 3420 or thefreq > 4180):
                double_check = 0
            sleep(0.01)
        double_check += 1
        thefreq = 2000
    print("Start")
    start = True
    return start

#-------------------------------------Movement Functions-------------------------------#
def Move_forward(delay, speed): #Move forward for a set time with speed 0-100
    GPIO.output(22,GPIO.HIGH)   #Set left motor to forward
    GPIO.output(10,GPIO.LOW)   #Set right motor to forward
    pwm_left.start(speed)    
    pwm_right.start(speed)
    time.sleep(delay*CONVCM)
    Stop(PAUSE)

def Move_forward_while_checking(delay, speed, line_detected): #Move forward for a set time with speed 0-100
    GPIO.output(22,GPIO.HIGH)   #Set left motor to forward
    GPIO.output(10,GPIO.LOW)   #Set right motor to forward
    pwm_left.start(speed)    #Start the left motor with a correction factor
    pwm_right.start(speed)
    for x in range (0,delay*10):
        time.sleep(.1*CONVCM)
        sensor0 = Find_short_dist(0)
        sensor1 = Find_short_dist(1)
        line_detected = Check_line(line_detected)
        if (line_detected):
            break
        if ((sensor0 < (MAXSHORT - 5)) or (sensor1 < (MAXSHORT - 5))):
            break
    Stop(PAUSE)
    return line_detected

def Move_forward_distance(clicks, speed):
    GPIO.output(22,GPIO.HIGH)   #Set left motor to forward
    GPIO.output(10,GPIO.LOW)   #Set right motor to forward
    pwm_left.start(speed)
    pwm_right.start(speed)
    right = 0
    while(right < clicks):
        GPIO.wait_for_edge(5, GPIO.FALLING)
        right += 1
    Stop(PAUSE)

def Move_forward_until_dist(distance, speed):  #Move robot forward until a distance from a wall
    sensor0 = Find_short_dist(0)
    sensor1 = Find_short_dist(1)
    escape = False   #If the bot moves forward don't have it move backward
    if (sensor0 > (distance) and sensor1 > (distance)):
        escape = True
    while (sensor0 > (distance) and sensor1 > (distance)):
        GPIO.output(22,GPIO.HIGH)   #Set left motor to forward
        GPIO.output(10,GPIO.LOW)   #Set right motor to forward
        pwm_left.start(speed)
        pwm_right.start(speed)
        sensor0 = Find_short_dist(0)
        sensor1 = Find_short_dist(1)
    if (escape):
        pwm_left.stop()
        pwm_right.stop()
        Stop(PAUSE)
        return None
    sensor0 = Find_short_dist(0)
    sensor1 = Find_short_dist(1)
    while (sensor0 < (distance) and sensor1 < (distance)):
        GPIO.output(22,GPIO.LOW)   #Set left motor to backward
        GPIO.output(10,GPIO.HIGH)   #Set right motor to backward
        pwm_left.start(speed)
        pwm_right.start(speed)
        sensor0 = Find_short_dist(0)
        sensor1 = Find_short_dist(1)
    pwm_left.stop()
    pwm_right.stop()
    Stop(PAUSE)
    
def Transition_forward(delay, speed):
    GPIO.output(22,GPIO.HIGH)   #Set left motor to forward
    GPIO.output(10,GPIO.LOW)    #Set right motor to forward
    pwm_left.start((speed/2)+2) #Start the left motor with a correction factor
    pwm_right.start(speed/2)
    time.sleep(delay*CONVCM)
    
def Transition_backward(delay, speed):
    GPIO.output(22,GPIO.LOW)   #Set left motor to forward
    GPIO.output(10,GPIO.HIGH)   #Set right motor to forward
    pwm_left.start((speed/2)+2)    #Start the left motor with a correction factor
    pwm_right.start(speed/2)
    time.sleep(delay*CONVCM)

def Move_backward(delay, speed): #Move forward for a set time with speed 0-100
    GPIO.output(22,GPIO.LOW)    #Set left motor to backward
    GPIO.output(10,GPIO.HIGH)    #Set right motor to backward
    pwm_left.start(speed)
    pwm_right.start(speed)
    time.sleep(delay*CONVCM)
    Stop(PAUSE)

def Stop(delay): #Stop the motors
    pwm_left.stop()
    pwm_right.stop()
    pwm_left.stop()
    pwm_right.stop()
    time.sleep(delay)

def Turn_right(angle,speed): #Turn right a specified angle
    GPIO.output(22,GPIO.HIGH)    #Set left motor to backward
    GPIO.output(10,GPIO.HIGH)   #Set right motor to forward
    pwm_left.start(speed)
    pwm_right.start(speed)
    time.sleep(angle*TURN)
    Stop(PAUSE)

def Turn_left(angle,speed): #Turn left a specified angle 
    GPIO.output(22,GPIO.LOW)   #Set left motor to forward
    GPIO.output(10,GPIO.LOW)    #Set right motor to backward   
    pwm_right.start(speed)
    pwm_left.start(speed)
    time.sleep(angle*TURN)
    Stop(PAUSE)

def Turn_until_aligned():
    sensor0 = Find_short_dist(0)    #Get front sensor values
    sensor1 = Find_short_dist(1)
    while (sensor0 < (sensor1 - ALIGNTOL) or sensor0 > (sensor1 + ALIGNTOL)):  #If the robot is misaligned
        angle = atan((abs(sensor0 - sensor1)/WIDTH))*(180.0/PI) #Find the angle robot should turn in degrees
        if (sensor0 < sensor1):             #Turn towards the smaller sensor value
            Turn_left(angle,TURNSPEED)
        if (sensor0 > sensor1):
            Turn_right(angle,TURNSPEED)
        sensor0 = Find_short_dist(0)        #Check front sensor values to see if they are aligned
        sensor1 = Find_short_dist(1)
    Stop(PAUSE)

def Align_on_line():
    x = 0
    if ((not GPIO.input(4)) and GPIO.input(15)):   #If the left line sensor is over a line and the right is not
        while(GPIO.input(15) and x<10000):     #While the right line sensor is not on a line
            GPIO.output(10,GPIO.LOW)   #Set right motor to forward
            pwm_right.start(SPEED/4)   #Turn on the right motor
            x += 1     #Increment a counter to prevent overshoot errors
        Stop(PAUSE)     #Stop the motors
    if (GPIO.input(4) and (not GPIO.input(15))):   #If the right line sensor is over a line and the left is not
        while(GPIO.input(4) and x < 10000):      #While the left line sensor is not on a line
            GPIO.output(22,GPIO.HIGH)   #Set left motor to forward
            pwm_left.start(SPEED/4)     #Turn on the left motor
            x += 1     #Increment a counter to prevent overshoot errors
        Stop(PAUSE)     #Stop the motors
    if (GPIO.input(4) and GPIO.input(15)):    #If neither sensor is above a line, assume it was overshot while turning
        while(GPIO.input(14) and x < 10):     #While the middle sensor doesn't see a line
            Transition_backward(.1,SPEED/2)   #Go backwards at 1/4th speed
            x += 1     #Increment a counter to prevent overshoot errors

def Right_wall_follow_until_door(inside_room, hall, dog_config):
    line_detected = False #Set this true when the line is detected
    Transition_forward(TRANS, SPEED)
    sensor_left_old = WALLDIST      #Initialize previous values as current values
    GPIO.output(22,GPIO.HIGH)   #Set left motor to forward
    GPIO.output(10,GPIO.LOW)    #Set right motor to forward
    while (not line_detected):
        #Find current sensor value and average 5 samples
        sensor2 = 0
        for x in range(0,5):
            sensor2 += Find_short_dist(2)
            line_detected = Check_line(line_detected)
            if (line_detected):   #Check for a line while checking sensors
                Stop(PAUSE)
                return line_detected, dog_config
        sensor_left = sensor2/5.0
        #Set the motor adjustment based on distance from target and previous distance
        proportional = -P*(WALLDIST - sensor_left)
        differential = D*(sensor_left - sensor_left_old)/T
        adjust_left = (proportional + differential)
        #Set the adjustment to be inside the duty cycle range
        if ((SPEED - adjust_left) <= 0 or (SPEED + adjust_left) <= 0):
            adjust_left = 0
        if ((SPEED + adjust_left) >= 100 or (SPEED - adjust_left) >=100):
            adjust_left = 50
        #Change the motor speeds according to the adjustment
        pwm_left.start(SPEED + adjust_left)
        pwm_right.start(SPEED - adjust_left)
        #Set the sensor_old to the current sensor values
        sensor_left_old = sensor_left
        #Check to see if the front sensor has dropped out of range, if so, turn right
        if (sensor_left > MAXSHORT):
            Transition_forward(TRANS, SPEED)
            for x in range (0,SPEED*14):
                Transition_forward(.01,SPEED*2)
                line_detected = Check_line(line_detected)
                if (line_detected):
                    Stop(PAUSE)
                    return line_detected, dog_config
            Stop(PAUSE)
            sensor2 = Find_short_dist(2)
            if (sensor2 > 20):
                Turn_right(95,TURNSPEED)
                Transition_forward(TRANS, SPEED)
                for x in range (0,SPEED*14):
                    Transition_forward(.01,SPEED*2)
                    line_detected = Check_line(line_detected)
                    if (line_detected):
                        Stop(PAUSE)
                        return line_detected, dog_config
                Stop(PAUSE)
            sensor_left_old = Find_short_dist(2)
        #Check to the middle sensor for a wall
        sensor3 = Find_short_dist(3)   #Get middle front sensor value
        if (sensor3 < (WALLDIST + .5)):      #See if a wall or obstacle has been spotted in front of robot
            obstacle = False
            if (hall == 3 or hall == 4 or room == 4):
                Move_backward(3,SPEED)
                obstacle = Scan_for_obstacle()
            if (inside_room and obstacle):
                Move_around_obstacle()
            if ((not inside_room) and obstacle):
                if (hall == 3):
                    dog_config = 1
                    break
                if (hall == 4):
                    dog_config = 2
                    break
                else:
                    dog_config = 3
                    break
            else:
                if (hall == 3 or hall == 4 or room == 4):
                    Move_forward(3,SPEED)
                Turn_left(90,TURNSPEED)
                Stop(PAUSE)
                GPIO.output(22,GPIO.HIGH)   #Set left motor to forward
                GPIO.output(10,GPIO.LOW)    #Set right motor to forward
                Transition_forward(TRANS, SPEED)
        #Check to see if a line is detected
        line_detected = Check_line(line_detected)
    for x in range (0, dog_config):
        GPIO.output(13, GPIO.HIGH) #Turn on Sound detect LED
        time.sleep(1)
        GPIO.output(13, GPIO.LOW) #Turn off Sound detect LED
    Stop(PAUSE)
    return line_detected, dog_config

def Right_wall_follow_until_turn(inside_room, hall, dog_config):
    line_detected = False #Set this true when the line is detected
    Transition_forward(TRANS, SPEED)
    sensor_left_old = WALLDIST      #Initialize previous values as current values
    GPIO.output(22,GPIO.HIGH)   #Set left motor to forward
    GPIO.output(10,GPIO.LOW)    #Set right motor to forward
    while (not line_detected):
        #Find current sensor value and average 5 samples
        sensor2 = 0
        for x in range(0,5):
            sensor2 += Find_short_dist(2)
        sensor_left = sensor2/5.0
        #Set the motor adjustment based on distance from target and previous distance
        proportional = -P*(WALLDIST - sensor_left)
        differential = D*(sensor_left - sensor_left_old)/T
        adjust_left = (proportional + differential)
        #Set the adjustment to be inside the duty cycle range
        if ((SPEED - adjust_left) <= 0 or (SPEED + adjust_left) <= 0):
            adjust_left = 0
        if ((SPEED + adjust_left) >= 100 or (SPEED - adjust_left) >=100):
            adjust_left = 50
        #Change the motor speeds according to the adjustment
        pwm_left.start(SPEED + adjust_left)
        pwm_right.start(SPEED - adjust_left)
        #Set the sensor_old to the current sensor values
        sensor_left_old = sensor_left
        #Check to the middle sensor for a wall
        sensor3 = Find_short_dist(3)   #Get middle front sensor value
        if (sensor3 < (WALLDIST + .5)):      #See if a wall or obstacle has been spotted in front of robot
            obstacle = False
            if (hall == 3 or hall == 4 or room == 4):
                Move_backward(3,SPEED)
                obstacle = Scan_for_obstacle()               
            if (inside_room and obstacle):
                Move_around_obstacle()
            if ((not inside_room) and obstacle):
                if (hall == 3):
                    dog_config = 1
                    break
                if (hall == 4):
                    dog_config = 2
                    break
                else:
                    dog_config = 3
                    break
            else:
                Stop(PAUSE)             #Once a wall is found stop to prevent current overload of motors
                Turn_until_aligned()                #Turn until aligned with wall
                Move_forward_until_dist(WALLDIST, SPEED) #Move to be WALLDIST away
                Turn_left(90,TURNSPEED) #Make a left turn
                break   #Once a left turn is found exit the wall follow
        #Check to see if a line is detected
        line_detected = Check_line(line_detected)
    for x in range (0, dog_config):
        GPIO.output(13, GPIO.HIGH) #Turn on Sound detect LED
        time.sleep(1)
        GPIO.output(13, GPIO.LOW) #Turn off Sound detect LED
    Stop(PAUSE)
    return line_detected, dog_config

def Right_wall_follow_for_distance(distance):
    Transition_forward(TRANS, SPEED)
    sensor_left_old = WALLDIST      #Initialize previous values as current values
    GPIO.output(22,GPIO.HIGH)   #Set left motor to forward
    GPIO.output(10,GPIO.LOW)    #Set right motor to forward
    distance_traveled = 0       #Set the current distance_traveled
    while (distance_traveled < distance):     #While the distance still hasn't been reached
        #Find current sensor value and average 5 samples
        sensor2 = 0
        for x in range(0,5):
            sensor2 += Find_short_dist(2)
        sensor_left = sensor2/5.0
        #Set the motor adjustment based on distance from target and previous distance
        proportional = -P*(WALLDIST - sensor_left)
        differential = D*(sensor_left - sensor_left_old)/T
        adjust_left = (proportional + differential)
        #Set the adjustment to be inside the duty cycle range
        if ((SPEED - adjust_left) <= 0 or (SPEED + adjust_left) <= 0):
            adjust_left = 0
        if ((SPEED + adjust_left) >= 100 or (SPEED - adjust_left) >=100):
            adjust_left = 50
        #Change the motor speeds according to the adjustment
        pwm_left.start(SPEED + adjust_left)
        pwm_right.start(SPEED - adjust_left)
        #Mirror detection
        if (sensor_left > MAXSHORT):
            Transition_forward(1,SPEED*2)
            distance_traveled += 1
        #Set the sensor_old to the current sensor values
        sensor_left_old = sensor_left
        distance_traveled += .1    #Add the distance traveled
    Stop(PAUSE)
    return line_detected, dog_config

def Move_around_obstacle():
    Turn_left(360,TURNSPEED)
    Stop(PAUSE)

def Enter_and_exit_room():
    Transition_forward(TRANS, SPEED)
    Move_forward(5, SPEED)
    Turn_left(180, TURNSPEED)
    sensor2 = Find_short_dist(2)
    if (sensor2 < (WALLDIST + 4)):
        Right_wall_follow_until_door(inside_room, hall, dog_config)
    else:
        Transition_forward(TRANS, SPEED)
        Move_forward(5, SPEED)

#-------------------------------------Sensor Functions---------------------------------#
def Check_line(line_detected):
    if ((not GPIO.input(4)) or (not GPIO.input(14)) or (not GPIO.input(15))):
            line_detected = True
    return line_detected

def Find_short_dist(sensor):
    voltage = mcp.read_adc(sensor)#*(3/1023) #Convert digital value to voltage
    if (voltage == 0):
        print ("Sensor out of range")
        voltage = .1
        #Linearize the sensor value to find distance
    voltage = voltage * 3.0
    voltage = voltage / 1023.0
    distance = (13.2/voltage) - 0.42
    return distance                         #return the distance in cm

def Find_long_dist(sensor):
    voltage = mcp.read_adc(sensor)*(3.0/1023) #Convert digital value to voltage
    if (voltage == 0):
        print ("Sensor out of range")
        voltage = .1
    distance = (60/voltage)
    return distance

def Scan_for_obstacle():   #Return True if the sensor is seeing an obstacle, False for a wall
    sensor3 = Find_short_dist(3)
    Turn_right(10,TURNSPEED)
    sensor1 = Find_short_dist(1)
    Turn_left(20,TURNSPEED)
    sensor0 = Find_short_dist(0)
    Turn_right(10,TURNSPEED)
    if ((sensor1 > (sensor3 + OBSTACLETOL)) or (sensor0 > (sensor3 + OBSTACLETOL))):
        print("Obstacle found")
        return True
    else:
        print("Wall found")
        return False

def Measure_room():
    sensor4_total = 0
    sensor5_total = 0
    sensor2_total = 0
    for x in range(0, 40):
        sensor4 = Find_long_dist(4)
        sensor5 = Find_long_dist(5)
        sensor2 = Find_short_dist(2)
        sensor4_total += sensor4
        sensor5_total += sensor5
        sensor2_total += sensor2
        Transition_backward(.1,SPEED)
    sensor4_avg = sensor4_total/40
    sensor5_avg = sensor5_total/40
    sensor2_avg = sensor2_total/40
    ###TEST
    print("Left: ", sensor4_avg)
    print("Right: ", sensor2_avg)
    print("Back: ", sensor5_avg)
    ###
    width = (sensor4_avg + sensor2_avg + WIDTH)
    length = (sensor5_avg + LENGTH + 3)       #Add 2 cm for line sensors sticking in front of robot
    area = (width)*(length)
    line_detected = False
    Move_forward_while_checking(5, SPEED/2, line_detected)
    ###TEST
    print("Width: ", width)
    print("Length: ", length)
    print("Area: ", area)
    Stop(PAUSE)
    return width, length, area

#------------------------------------Position Functions--------------------------------#

def Determine_room(width, length, area):
    room = 0
    if (area < 4917):
        room = 4
        return room
    if (area >= 4917 and area < 8000 and width < 85):
        room = 1
        return room
    if (area >= 4917 and area < 8000 and width >= 85):
        room = 2
        return room
    if (area >= 8000):
        room = 3
        return room
    print ("Error with long distance sensors")
    return room

def Determine_entrance(room):
    entrance = 0
    sensor2 = Find_short_dist(2)   #Get front right sensor
    sensor4 = Find_long_dist(4)    #Get left sensor
    sensor5 = Find_long_dist(5)    #Get back sensor
    if (room == 3 and sensor2 < MAXSHORT and sensor4 < (110 - (WIDTH + WALLDIST))):
        entrance = 2  #room 3 left entrance is at the top
    if (room == 3 and sensor2 >= MAXSHORT):
        entrance = 1  #room 3 left entrance is at the bottom
    if (room == 3 and sensor2 < MAXSHORT and sensor4 >= (110 - (WIDTH + WALLDIST))):
        entrance = 3  #room 3 top entrance
    if (room == 4):
        Move_forward_until_dist(WALLDIST,SPEED/2) #Make sure the robot is still WALLDIST away
        Turn_until_aligned()                    #Turn until aligned with the wall
        Move_forward_until_dist(WALLDIST,SPEED/2) #Make sure the robot is still WALLDIST away
        Turn_left(90,TURNSPEED)
        sensor5 = Find_long_dist(5)
        if (sensor5 < 70):
            entrance = 5   #room 4 top entrance            
            GPIO.output(6, GPIO.HIGH) #Turn on Sound detect LED
            time.sleep(1)
            GPIO.output(6, GPIO.LOW) #Turn off Sound detect LED
        else:
            entrance = 4   #room 4 bottom entrance
    return entrance

def Determine_hall(room, entrance):
    hall = 0   #Catch all if room is not set
    if (room == 1):
        hall = 1
    if (room == 2):
        hall = 2
    if (room == 3 and (entrance == 1 or entrance == 2)):
        hall = 3
    if (room == 3 and entrance == 3):
        hall = 4
    if (room == 4 and entrance == 4):
        hall = 6
    if (room == 4 and entrance == 5):
        hall = 7
    return hall

def Set_config(room3_config, room4_config, dog_config, entrance):   #Set which configuration the maze is in
    if (entrance == 1):
        room3_config = 1
    if (entrance == 2):
        room3_config = 2
    if (entrance == 4):
        room4_config = 1
    if (entrance == 5):
        room4_config = 2
    return room3_config, room4_config, dog_config

def Determine_position(room, hall, room3_config, room4_config, dog_config, entrance):
    if (room == 1 or hall == 1):
        position = 1
    if (room == 2 or hall == 2):
        position = 2
    if ((room == 3 and (entrance == 1 or entrance == 2)) or hall == 3):
        position = 3
    if (room == 3 and entrance == 3):
        position = 4
    if (room == 4 and entrance == 5):
        position = 5
    if ((room == 4 and entrance == 4) or hall == 6):
        position = 6
    return position
#--------------------------------Fire Extinguishing Functions-------------------------#

def Detect_flame():
    fire_check = 0
    for x in range(0,10000):
        fire_check += GPIO.input(12)
    if (fire_check < 9990):
        print ("Fire Detected")
        fire_detected = True
    else:
        print ("Fire not detected")
        fire_detected = False
    return fire_detected

def Extinguish_flame(fire_detected):
    Stop(.1)              #Stop the robot while extinguishing
    return_angle = 0      #Initialize a value for how much the robot will turn so it can correct itself
    return_dist = 0       #Initialize a value for how far the robot will travel so it can correct itself
    while (fire_detected):
        GPIO.output(6, GPIO.HIGH)    #Turn on Fire detect LED
        location, sensor6 = Search_for_flame()
        pwm_servo.start(location)     #Move the servo to the face the flame 
        if (sensor6 < 750):           #If the flame is close enough to the robot
            time.sleep(2)
            GPIO.output(16, GPIO.HIGH)#Open the CO2 canister
            time.sleep(EXTINGUISH)    #Hold open for a delay
            GPIO.output(16, GPIO.LOW) #Close the CO2 canister
            GPIO.output(6, GPIO.LOW) #Turn off Fire detect LED
        else:
            if(location > 12):
                Turn_left(((location-1)-12)*9,TURNSPEED)
                return_angle += ((location-1)-12)*9
                print ("Angle: ", ((location-1)-12)*9)
            if(location < 12):
                Turn_right((12-(location-1))*9,TURNSPEED)
                return_angle -= (12-(location-1))*9
                print ("Angle: ", ((location-1)-12)*9)
            Transition_forward(TRANS,SPEED)
            forward = (((sensor6-700)/10)**2)/150
            if (sensor6 < 900 and (location > 16 or location < 8)):
                forward = forward/3      #If the sensor is close to the fire but still angled wrong, advance slowly
            print("forward:", forward)
            Move_forward(forward,SPEED)
            return_dist += forward
        fire_detected = Detect_flame()
    return return_dist, return_angle

def Search_for_flame():   #A function using a phototransitor and servo to fine exact flame location
    Stop(.1)              #Stop the robot while searching
    #double_check = True   #Set a double check boolean to ensure proper positioning
    location_old = 1000   #Set a temporary location of the flame
    #while(double_check):  #While the function has not found the same value twice
    sensor6_temp = 10000            #Set a temporary phototransitor value
    pwm_servo.start(1)              #Start the servo at the far right side
    time.sleep(1)                   #Take a short pause
    for x in range (1,20):          #Sweep the servo right to left recording values on the way
        sensor6 = mcp.read_adc(6)
        print (sensor6)
        pwm_servo.start(x)
        time.sleep(.2)
        if (sensor6 < sensor6_temp):#Record the minimum value
            sensor6_temp = sensor6
            location = x - 1          #Set location to that value 
        #if (location == location_old):  #Set the double_check if the same value was obtained twice
        #    double_check = False
        #else:                           #Otherwise set the current location in the temporary location
        #    location_old = location
    print(location)
    return location, sensor6_temp       #Return the position the robot should move its extinguisher to and the strength of the flame
    
def Search_room(room, fire_extinguished, inside_room, hall, dog_config):
    line_detected = False
    fire_detected = Detect_flame()       #Use UV Tron to check for a fire
    Transition_forward(TRANS*3, SPEED)
    Move_forward(9, SPEED)
    if (fire_detected):              #If a fire was spotted
        return_dist, return_angle = Extinguish_flame(fire_detected)  #Extinguish the fire
        fire_extinguished = True         #Set the fire to be extinguished
        Transition_backward(TRANS, SPEED)
        Move_backward(return_dist*.8,SPEED)
        if(return_angle < 0):
            Turn_left(return_angle*-1,TURNSPEED)
        if(return_angle >= 0):
            Turn_right(return_angle,TURNSPEED)
    sensor2 = Find_short_dist(2)
    #If at the bottom entrance of room 3 do some additional aligning
    if (sensor2 < WALLDIST + 6):
        Turn_right(90,TURNSPEED)
        Turn_until_aligned()
        Turn_left(155,TURNSPEED)
        fire_detected = Detect_flame()       #Use UV Tron to check for a fire
        if (fire_detected):                  #If a fire was spotted
            return_dist, return_angle = Extinguish_flame(fire_detected)  #Extinguish the fire
            fire_extinguished = True         #Set the fire to be extinguished
            Transition_backward(TRANS, SPEED)
            Move_backward(return_dist*.8,SPEED)
            if(return_angle < 0):
                Turn_left(return_angle*-1,TURNSPEED)
            if(return_angle >= 0):
                Turn_right(return_angle,TURNSPEED)
    #Otherwise just turn left
    else:
        Turn_right(90,TURNSPEED)
        fire_detected = Detect_flame()       #Use UV Tron to check for a fire
        if (fire_detected):                  #If a fire was spotted
            return_dist, return_angle = Extinguish_flame(fire_detected)  #Extinguish the fire
            fire_extinguished = True         #Set the fire to be extinguished
            Transition_backward(TRANS, SPEED)
            Move_backward(return_dist*.8,SPEED)
            if(return_angle < 0):
                Turn_left(return_angle*-1,TURNSPEED)
            if(return_angle >= 0):
                Turn_right(return_angle,TURNSPEED)
        Turn_left(180,TURNSPEED)
    Move_forward_until_dist(WALLDIST,SPEED/2)
    Turn_until_aligned()
    Move_forward_until_dist(WALLDIST,SPEED/2)
    Turn_left(90,TURNSPEED)
    line_detected, dog_config = Right_wall_follow_until_door(inside_room, hall, dog_config)
    if (line_detected):
        Align_on_line()
    return fire_extinguished
    ###
    #Code for searching for candle and extinguishing it
    #return fire_extinguished
    ###

#------------------------------------High Level Functions-----------------------------#
            
def Navigate_out_of_room(inside_room, hall, room3_config, room4_config, dog_config):
    Move_forward_until_dist(WALLDIST, SPEED)
    Turn_left(22.5, TURNSPEED)
    Turn_until_aligned()
    Move_forward_until_dist(WALLDIST, SPEED)
    Turn_left(90, TURNSPEED)
    line_detected, dog_config = Right_wall_follow_until_door(inside_room, hall, dog_config)
    width, length, area = Measure_room()
    room = Determine_room(width, length, area)
    entrance = Determine_entrance(room)
    room3_config, room4_config, dog_config = Set_config(room3_config, room4_config, dog_config, entrance)
    return room, entrance, room3_config, room4_config

#-----------------------------------------Main Loop-----------------------------------#


###PROGRAMMING MODE
sensor2 = Find_short_dist(2)
if (sensor2 < 8):
    while(True):
        print ("Programming mode")
        Stop(1000)

#SIGNAL DETECTION
#Wait for 3.8 kHz singal
GPIO.output(6, GPIO.HIGH) #Turn on Sound detect LED
time.sleep(1)
GPIO.output(6, GPIO.LOW) #Turn off Sound detect LED
start = Sound_start()
GPIO.output(13, GPIO.HIGH) #Turn on Sound detect LED
time.sleep(1)
GPIO.output(13, GPIO.LOW) #Turn off Sound detect LED

#NAVIGATING OUT OF FIRST ROOM
#Initialize the motor speed
Move_forward(.1,SPEED)
Stop(PAUSE)
#Check if a there is a wall in front of robot
while (start):
    sensor0 = Find_short_dist(0)   #Get right front sensor
    sensor1 = Find_short_dist(1)   #Get left front sensor
    room_not_found = 0             #Set a variable to prevent an infinite loop while searching for room
    #First navigate from the arbitrary starting room
    while(inside_room):
        if (sensor0 < MAXSHORT and sensor1 < MAXSHORT):
            room, entrance, room3_config, room4_config = Navigate_out_of_room(inside_room, hall, room3_config, room4_config, dog_config) #Navigate from the first room to a door
            if(line_detected):          #If a line was detected
                Align_on_line()         #Align on the line
                line_detected = False   #Reset the line sensors
            room_checked[room - 1] = True   #Mark off this room as checked for the candle
            inside_room = False         #Tell the bot it's outside the room
            starting_room = room        #Remeber which room the robot began in so it can return there
            hall = Determine_hall(room, entrance) #Decide which hall section the robot is in
        else:
            Turn_left(45,TURNSPEED)             #Turn left 45 degrees
            room_not_found += 1                 #Increment a counter to prevent from spinning in circles
            sensor0 = Find_short_dist(0)        #Check sensors again
            sensor1 = Find_short_dist(1)
        if (room_not_found > 7):                #If a wall is still not found after checking all 8 directions
            Transition_forward(TRANS,SPEED)
            line_detected = Move_forward_while_checking(10,SPEED,line_detected)  #Move forward looking for a line or wall
            sensor0 = Find_short_dist(0)   #Get right front sensor
            sensor1 = Find_short_dist(1)   #Get left front sensor
            if(line_detected):                  #If a line was detected
                Align_on_line()                 #Align on the line
                line_detected = False           #Reset the line sensors
                inside_room = False             #Tell the robot its outside the room
                width, length, area = Measure_room() #Measure the room
                room = Determine_room(width, length, area) #Determine which room it is in
                starting_room = room            #Remeber which room the robot began in so it can return there
                entrance = Determine_entrance(room)   #Set the entrance
                hall = Determine_hall(room, entrance) #Decide which hall section the robot is in
                room3_config, room4_config, dog_config = Set_config(room3_config, room4_config, dog_config, entrance)  #Set the configuration
                room_checked[room - 1] = True   #Mark off this room as checked for the candle
            room_not_found = 2                  #Reset the robot to do 270 next check
            
    position = Determine_position(room, hall, room3_config, room4_config, dog_config, entrance)   #Determine where in the maze the bot is
    while (not fire_extinguished or not (room == starting_room)):
        print("Position is: ", position)
        print("Hall is: ", hall)
        if (position == 1):   #If exiting room 1
            Transition_forward(TRANS*3, SPEED)  #Transition forward far enough to go off line
            line_detected, dog_config = Right_wall_follow_until_door(inside_room, hall, dog_config)  #Follow until the next room
            if(line_detected):                  #If a line was detected
                Align_on_line()                 #Align on the line
                line_detected = False           #Reset the line sensors
            inside_room = True                  #Tell the bot it's outside the room
            room = 2                            #Set which room bot is in
            Search_room(room, fire_extinguished, inside_room, hall, dog_config)     #Search the room the bot is in
            room_checked[room - 1] = True       #Mark off this room as checked for the candle
            inside_room = False                 #Mark the bot is back in the hall
            hall = 2                            #Set the hall to be 2
            position = 2                        #Set the new position
        if (position == 2):  #If exiting room 2
            Transition_forward(TRANS*3, SPEED)  #Transition forward far enough to go off line
            line_detected, dog_config = Right_wall_follow_until_door(inside_room, hall, dog_config)  #Follow until the next room
            if(line_detected):                  #If a line was detected
                Align_on_line()                 #Align on the line
                line_detected = False           #Reset the line sensors
            inside_room = True                  #Tell the bot it's outside the room
            room = 3                            #Set which room bot is in
            Search_room(room, fire_extinguished, inside_room, hall, dog_config)     #Search the room the bot is in
            room_checked[room - 1] = True           #Mark off this room as checked for the candle
            inside_room = False                 #Mark the bot is back in the hall
            hall = 3                            #Set the hall to 3
            sensor2 = Find_short_dist(2)        #Check the right front sensor
            if (sensor2 < (WALLDIST + 5)):      #If there is a wall to the right of the bot
                entrance = 2                    #Set the entrance to be the left upper entrance of room 3
            else:                               #Otherwise
                entrance = 1                    #Set the entrance to be the lower left entance
            position = 3                        #Set the new position
        if (position == 3):  #If exiting room 3 on the left side
            Transition_forward(TRANS*3, SPEED)  #Transition forward far enough to go off line
            if (entrance == 1):                 #If the robot is at the bottom entrance of room 3
                Move_forward(3, SPEED/2)        #Move forward some to account for earlier right wall drop off 
            line_detected, dog_config = Right_wall_follow_until_door(inside_room, hall, dog_config)  #Follow until the next room
            if (line_detected):                 #If there is no dog in hall 3, and robot goes to entrance 3
                Align_on_line()                 #Align on the line
                line_detected = False           #Reset the line sensors
                Turn_left(90, TURNSPEED)        #Turn around to face hall 4
                Move_forward_until_dist(WALLDIST, SPEED/2) #Move to the wall of hall 4
                Turn_until_aligned()            #Turn until aligned to the wall
                Move_forward_until_dist(WALLDIST, SPEED/2) #Move to the wall of hall 4
                Turn_left(90,TURNSPEED)         #Turn to look down hall 4
                hall = 4                        #Set the hall to 4
                position = 4                    #Set the new position
            else:                               #If there is a dog in hall 3
                Turn_left(90, TURNSPEED)        #Turn left 90 degrees
                line_detected = Move_forward_while_checking(20,SPEED,line_detected)   #Go forward to look for room 4
                if (not line_detected):         #If lower entrance of room 4 isn't found
                    hall = 8                    #Set the hall to 8
                    Turn_left(90, TURNSPEED)    #Turn left to align with room 4 bottom wall
                    line_detected, dog_config = Right_wall_follow_until_door(inside_room, hall, dog_config)  #Follow until the next room
                    if(line_detected):                  #If a line was detected
                        Align_on_line()                 #Align on the line
                        line_detected = False           #Reset the line sensors
                        inside_room = True                  #Tell the bot it's outside the room
                        room = 4                            #Set which room bot is in
                        Search_room(room, fire_extinguished, inside_room, hall, dog_config)     #Search the room the bot is in
                        room_checked[room - 1] = True       #Mark off this room as checked for the candle
                        inside_room = False                 #Mark the bot is back in the hall
                        hall = 9                            #Set the hall to 9
                        entrance = 5                        #Set the entrance to be the top entrance of room 4
                        position = 5                        #Set the new position to the top of room 4                 
                if (line_detected):                     #If the entrance to room 4 was found
                    Align_on_line()                     #Align on the line
                    line_detected = False               #Reset the line sensors
                    inside_room = True                  #Tell the bot it's outside the room
                    room = 4                            #Set which room bot is in
                    Search_room(room, fire_extinguished, inside_room, hall, dog_config)     #Search the room the bot is in
                    room_checked[room - 1] = True       #Mark off this room as checked for the candle
                    inside_room = False                 #Mark the bot is back in the hall
                    hall = 6                            #Set the hall to 6
                    entrance = 4                        #Set the entrance to be the lower entrance of room 4
                    position = 6                        #Set the new position
        if (position == 4):                     #If the robot is at the top entrance of room 3
            Transition_forward(TRANS*3, SPEED)  #Transition forward far enough to go off line
            line_detected, dog_config = Right_wall_follow_until_turn(inside_room, hall, dog_config)  #Follow until the top wall
            if (dog_config == 2):               #If there is a dog in hall 4
                Turn_left(90,TURNSPEED)         #Turn left 90 degrees
                Move_forward_while_checking(3,SPEED,line_detected)  #Move forward looking for a wall
                Turn_left(90,TURNSPEED)         #Turn left 90 degrees
                line_detected, dog_config = Right_wall_follow_until_turn(inside_room, hall, dog_config)  #Follow until the top wall
                if (line_detected):                     #If the top entrance to room 4 was found
                    Align_on_line()                     #Align on the line
                    line_detected = False               #Reset the line sensors
                    inside_room = True                  #Tell the bot it's outside the room
                    room = 4                            #Set which room bot is in
                    Search_room(room, fire_extinguished, inside_room, hall, dog_config)     #Search the room the bot is in
                    room_checked[room - 1] = True       #Mark off this room as checked for the candle
                    inside_room = False                 #Mark the bot is back in the hall
                    entrance = Determine_entrance(room)
                if (entrance == 4):
                    position = 6
                    outside_room4 = True
                else:
                    position = 5
                    outside_room4 = True
            else:
                Right_wall_follow_for_distance(60)  #Follow the right wall for 60 cm
                Turn_left(90,TURNSPEED)             #Turn right 90 degrees
                line_detected = Move_forward_while_checking(10,SPEED,line_detected)  #Move forward looking for a line or wall
                if (line_detected):                     #If the top entrance to room 4 was found
                    Align_on_line()                     #Align on the line
                    line_detected = False               #Reset the line sensors
                    inside_room = True                  #Tell the bot it's outside the room
                    room = 4                            #Set which room bot is in
                    Search_room(room, fire_extinguished, inside_room, hall, dog_config)     #Search the room the bot is in
                    room_checked[room - 1] = True       #Mark off this room as checked for the candle
                    inside_room = False                 #Mark the bot is back in the hall
                    hall = 7                            #Set the hall to 7
                    entrance = 5                        #Set the entrance to be the upper entrance of room 4
                    position = 5                        #Set the new position
                else:                                   #If room 4 entrance is on bottom
                    Turn_left(90,TURNSPEED)             #Turn left 90 degrees
                    hall = 7                            #Set the hall to 7
                    position = 5                        #Set the new position
        if (position == 5):                     #If the robot is at the top entrance of room 4
            if (room_checked[3] and (not dog_config == 3)):     #If the robot began in room 4 and there isn't a dog in the top hall
                Transition_forward(TRANS*3, SPEED)      #Move off the line
                if(not(starting_room == 4 or outside_room4)):  
                    Move_forward_while_checking(30,SPEED,line_detected)  #Move forward looking for a wall
                    Turn_until_aligned()                    #Turn until aligned with the wall
                    Move_forward_until_dist(WALLDIST,SPEED/2) #Make sure the robot is still WALLDIST away
                    Turn_left(90,TURNSPEED)                 #Turn left 90 degrees
                line_detected, dog_config = Right_wall_follow_until_door(inside_room, hall, dog_config)  #Follow until the next room
                if (line_detected):                     #If the top entrance to room 4 was found
                    Align_on_line()                     #Align on the line
                    line_detected = False               #Reset the line sensors
                    inside_room = True                  #Tell the bot it's outside the room
                    room = 1                            #Set which room bot is in
                    Search_room(room, fire_extinguished, inside_room, hall, dog_config)     #Search the room the bot is in
                    room_checked[room - 1] = True       #Mark off this room as checked for the candle
                    inside_room = False                 #Mark the bot is back in the hall
                    hall = 1                            #Set the hall to 1
                    position = 1                        #Set the new position
                else:
                    Turn_left(90,TURNSPEED)             #Turn left to face room 4 again
                    Move_forward_while_checking(200,SPEED,line_detected)  
                    line_detected = Move_forward_while_checking(10,SPEED,line_detected)  #Move forward looking for a wall or lione
                    if (line_detected):                     #If the top entrance to room 4 was found
                        Align_on_line()                     #Align on the line
                        line_detected = False               #Reset the line sensors
                        Search_room(room, fire_extinguished, inside_room, hall, dog_config)
                        position = 5
                        dog_config = 3
                    else:
                        Right_wall_follow_until_door(inside_room, hall, dog_config)  #Follow until door
                        Align_on_line()                     #Align on the line
                        Search_room(room, fire_extinguished, inside_room, hall, dog_config)
                        position = 5
                        dog_config = 3
            else:
                Transition_forward(TRANS*3, SPEED)      #Transition forward far enough to go off line
                if(starting_room == 4 and (not dog_config == 3)):                 #If robot started in room 4
                    Turn_right(90,TURNSPEED)            #Turn right to avoid driving into top wall
                line_detected, dog_config = Right_wall_follow_until_door(inside_room, hall, dog_config)  #Follow until the next room
                if (line_detected):                     #If the entrance to room 4 was found
                    Align_on_line()                     #Align on the line
                    line_detected = False               #Reset the line sensors
                    inside_room = True                  #Tell the bot it's outside the room
                    room = 4                            #Set which room bot is in
                    Search_room(room, fire_extinguished, inside_room, hall, dog_config)     #Search the room the bot is in
                    room_checked[room - 1] = True       #Mark off this room as checked for the candle
                    inside_room = False                 #Mark the bot is back in the hall
                    hall = 6                            #Set the hall to 6
                    entrance = 4                        #Set the entrance to be the lower entrance of room 4
                    position = 6                        #Set the new position
                else:
                    Turn_left(180,TURNSPEED)            #Do a 180
                    Move_forward_while_checking(20,SPEED,line_detected)   #Move to right outer wall
                    Turn_left(90,TURNSPEED)
                    line_detected, dog_config = Right_wall_follow_until_door(inside_room, hall, dog_config)  #Follow until the next room
                    Align_on_line()                     #Align on the line
                    line_detected = False               #Reset the line sensors
                    inside_room = True                  #Tell the bot it's outside the room
                    room = 1                            #Set which room bot is in
                    Search_room(room, fire_extinguished, inside_room, hall, dog_config)     #Search the room the bot is in
                    room_checked[room - 1] = True       #Mark off this room as checked for the candle
                    inside_room = False                 #Mark the bot is back in the hall
                    hall = 1                            #Set the hall to 1
                    position = 1                        #Set the new position
        if (position == 6):                             #If the robot is at the bottom of room 4
            if(starting_room == 4 or outside_room4):
                Turn_right(190,TURNSPEED)
            else:
                Transition_forward(TRANS,SPEED)
                Move_forward_while_checking(30,SPEED,line_detected)  #Move forward looking for a wall
                Move_forward_until_dist(WALLDIST,SPEED/2)   #Make sure the robot is still WALLDIST away
                Turn_until_aligned()                        #Turn until aligned with the wall
                Move_forward_until_dist(WALLDIST,SPEED/2)   #Make sure the robot is still WALLDIST away
                Turn_right(100,TURNSPEED)                   #Turn right 100 degrees to face the center of the intersection
            Transition_forward(TRANS,SPEED)             #Transition forward slowly
            Move_forward(12,SPEED)                      #Move forward 12 cm
            Turn_right(95,TURNSPEED)                    #Turn down hallway 8
            Transition_forward(TRANS,SPEED)             #Transition forward slowly
            Move_forward(6,SPEED)                       #Move forward to make sure the right wall of room 4 is there
            Right_wall_follow_for_distance(30)          #Follow up this wall for 30 cm
            Turn_left(90,TURNSPEED)                     #Turn left to face room 1
            Move_forward_until_dist(WALLDIST,SPEED/2)   #Make sure the robot is still WALLDIST away
            Turn_until_aligned()                        #Turn until aligned with the wall
            Move_forward_until_dist(WALLDIST,SPEED/2)   #Make sure the robot is still WALLDIST away
            Turn_left(90,TURNSPEED)                     #Turn left to put the wall on the right side of the robot
            line_detected, dog_config = Right_wall_follow_until_door(inside_room, hall, dog_config)  #Follow until the next room
            if (line_detected):                     #If the  entrance to room 1 was found
                Align_on_line()                     #Align on the line
                line_detected = False               #Reset the line sensors
                inside_room = True                  #Tell the bot it's outside the room
                room = 1                            #Set which room bot is in
                Search_room(room, fire_extinguished, inside_room, hall, dog_config)     #Search the room the bot is in
                room_checked[room - 1] = True       #Mark off this room as checked for the candle
                inside_room = False                 #Mark the bot is back in the hall
                hall = 1                            #Set the hall to 1
                position = 1                        #Set the new position
            
        if (inside_room):    #If there is an error and the robot is stuck in a room
            room, entrance, room3_config, room4_config = Navigate_out_of_room(inside_room, hall, room3_config, room4_config, dog_config) #Get out of the room
            room_checked[room - 1] = True         #Mark off this room as checked for the candle
            inside_room = False                   #Mark bot is outside of a room
            hall = Determine_hall(room, entrance) #Decide which hall section the robot is in
            Determine_position(room, hall, room3_config, room4_config, entrance)   #Determine where in the maze the bot is
    
    Transition_backward(TRANS,SPEED)  #Once returned to starting room, back into room a little
    Move_backward(3,SPEED)

#-----------------------------------------Test Loop-----------------------------------#

print("Move forward")
Move_forward(.1,SPEED)
Stop(PAUSE)
test = 100  #Edit for which test loop you want to run, position tests start at 14 for position 1
#Test for distance sensors
if (test == 0):
    while(True):
        sensor_value = mcp.read_adc(0)
        sensor0 = Find_short_dist(0)
        sensor1 = Find_short_dist(1)
        sensor2 = Find_short_dist(2)
        sensor3 = Find_short_dist(3)
        sensor4 = Find_long_dist(4)
        sensor5 = Find_long_dist(5)
        print (int(sensor0), int(sensor1), int(sensor2), int(sensor3), int(sensor4), int(sensor5))
        print("Value is:", sensor_value)
        print("Front Left is:", int(sensor0), "cm")
        print("Front Right is:",int(sensor1), "cm")
        time.sleep(2)

#Test for basic movement
if (test == 1):
    Stop(3)
    print("Move backward")
    Move_backward(10,SPEED)   
    print("Move forward")
    Move_forward(10,SPEED) 
    print("Turn right 90 degrees")
    Turn_right(270,TURNSPEED)
    print("Turn left 90 degrees")
    Turn_left(270,TURNSPEED)
    print("Move backward")
    Move_backward(10,SPEED) 
    print("Move forward")
    Move_forward(10,SPEED)
 

#Test for beginning algorithm
if(test == 2):
    print("Move forward until 10cm from a wall")
    Move_forward_until_dist(10,SPEED)
    print("Turn 20 degrees to the left")
    Turn_left(20,TURNSPEED)
    print("Turn until aligned")
    Turn_until_aligned()
    print("Move forward until 10cm from a wall")
    Move_forward_until_dist(10,SPEED)
    print("Turn left 90 degrees")
    Turn_left(90,TURNSPEED)
    print("Right wall follow until door")
    Right_wall_follow_until_door(inside_room, hall, dog_config)
    print("Measure the room")
    width, length, area = Measure_room()
    print("The length of this room is %d, the width of this room is %d, and the area is %d" %(width, length, area))
    entrance = Determine_entrance(room)
    print("The entrance is: ", entrance)

if(test == 3):
    room, entrance, room3_config, room4_config = Navigate_out_of_room(inside_room, hall, room3_config, room4_config, dog_config) #Navigate from the first room to a door
    print("Room: ", room)
    print("Entrance: ", entrance)
    Stop(5)
    
if(test == 4):
    print('| {0:>4} | {1:>4} | {2:>4} | {3:>4} | {4:>4} | {5:>4} | {6:>4} | {7:>4} |'.format(*range(8)))
    print('----------------------------------------------------------')
    # Main program loop.
    while True:
        # Read all the ADC channel values in a list.
        print (mcp.read_adc(1))

        values = [0]*8
        for i in range(8):
            # The read_adc function will get the value of the specified channel (0-$
            values[i] = mcp.read_adc(i)
        # Print the ADC values.
        print('| {0:>4} | {1:>4} | {2:>4} | {3:>4} | {4:>4} | {5:>4} | {6:>4} | {7:>4} |'.format(*values))
        # Pause for half a second.
        time.sleep(0.5)
        
if (test == 5):
    print("Move forward")
    Move_forward(60,SPEED)
    
if (test == 6):
    Turn_until_aligned()

if (test == 7):
    width,length,area = Measure_room()
    print("Width: ",width)
    print("Length: ", length)
    print("Area: ", area)
    room = Determine_room(width, length, area)
    print("Room: ", room)

if (test == 8):
    Move_forward_until_dist(WALLDIST, SPEED)
    
if (test == 9):
    for x in range (0,20):
        pwm_servo.start(x)
        time.sleep(1)
        print(x)

if (test == 10):
    location = Search_for_flame()
    print (location)
    pwm_servo.start(location)
    time.sleep(2)

if (test == 11):
    while(True):
        GPIO.output(16, GPIO.HIGH)
        time.sleep(1)
        print("high")
        GPIO.output(16, GPIO.LOW)
        time.sleep(1)
        print("low")

if (test == 12):
    room, entrance, room3_config, room4_config = Navigate_out_of_room(inside_room, hall, room3_config, room4_config, dog_config)
    print("Room is: ", room)
    if (room == 1):
        Move_forward(5,SPEED/2)
    elif (room == 2):
        Turn_right(90,TURNSPEED)
    elif (room == 3):
        Turn_left(90,TURNSPEED)
    elif (room == 4):
        Move_backward(5,SPEED/2)
    else:
        Turn_left(180,TURNSPEED)

if (test == 13):
    hall = 3
    line_detected, dog_config = Right_wall_follow_until_door(inside_room, hall, dog_config)
    if(line_detected):
        Align_on_line()
        line_detected = False
        print ("Aligned")

if (test == 14):
    position = 1
    hall = 1
    if (position == 1):   #If exiting room 1
        Transition_forward(TRANS*3, SPEED)  #Transition forward far enough to go off line
        line_detected, dog_config = Right_wall_follow_until_door(inside_room, hall, dog_config)  #Follow until the next room
        if(line_detected):                  #If a line was detected
            Align_on_line()                 #Align on the line
            line_detected = False           #Reset the line sensors
        inside_room = True                  #Tell the bot it's outside the room
        room = 2                            #Set which room bot is in
        Search_room(room, fire_extinguished, inside_room, hall, dog_config)     #Search the room the bot is in
        room_checked[room - 1] = True       #Mark off this room as checked for the candle
        inside_room = False                 #Mark the bot is back in the hall
        hall = 2                            #Set the hall to be 2
        position = 2                        #Set the new position
            
if (test == 15):
    position = 2
    hall = 2
    if (position == 2):  #If exiting room 2
            Transition_forward(TRANS*3, SPEED)  #Transition forward far enough to go off line
            line_detected, dog_config = Right_wall_follow_until_door(inside_room, hall, dog_config)  #Follow until the next room
            if(line_detected):                  #If a line was detected
                Align_on_line()                 #Align on the line
                line_detected = False           #Reset the line sensors
            inside_room = True                  #Tell the bot it's outside the room
            room = 3                            #Set which room bot is in
            Search_room(room, fire_extinguished, inside_room, hall, dog_config)     #Search the room the bot is in
            room_checked[room - 1] = True       #Mark off this room as checked for the candle
            inside_room = False                 #Mark the bot is back in the hall
            hall = 3                            #Set the hall to 3
            sensor2 = Find_short_dist(2)        #Check the right front sensor
            if (sensor2 < (WALLDIST + 5)):      #If there is a wall to the right of the bot
                entrance = 2                    #Set the entrance to be the left lower entrance of room 3
            else:                               #Otherwise
                entrance = 1                    #Set the entrance to be the upper left entance
            position = 3                        #Set the new position

if (test == 16):
    position = 3
    hall = 3
    entrance = 1
    if (position == 3):  #If exiting room 3 on the left side
            Transition_forward(TRANS*3, SPEED)  #Transition forward far enough to go off line
            line_detected, dog_config = Right_wall_follow_until_door(inside_room, hall, dog_config)  #Follow until the next room
            if (entrance == 1):                 #If the robot is at the bottom entrance of room 3
                Move_forward(3, SPEED/2)        #Move forward some to account for earlier right wall drop off 
            if (line_detected):                 #If there is no dog in hall 3, and robot goes to entrance 3
                Align_on_line()                 #Align on the line
                line_detected = False           #Reset the line sensors
                Turn_left(90, TURNSPEED)        #Turn around to face hall 4
                Move_forward_until_dist(WALLDIST, SPEED/2) #Move to the wall of hall 4
                Turn_until_aligned()            #Turn until aligned to the wall
                Move_forward_until_dist(WALLDIST, SPEED/2) #Move to the wall of hall 4
                Turn_left(90,TURNSPEED)         #Turn to look down hall 4
                hall = 4                        #Set the hall to 4
                position = 4                    #Set the new position
            else:                               #If there is a dog in hall 3
                Turn_left(90, TURNSPEED)        #Turn left 90 degrees
                line_detected = Move_forward_while_checking(20,SPEED,line_detected)   #Go forward to look for room 4
                if (not line_detected):         #If lower entrance of room 4 isn't found
                    hall = 8                    #Set the hall to 8
                    line_detected, dog_config = Right_wall_follow_until_door(inside_room, hall, dog_config)  #Follow until the next room
                    if(line_detected):                  #If a line was detected
                        Align_on_line()                 #Align on the line
                        line_detected = False           #Reset the line sensors
                    inside_room = True                  #Tell the bot it's outside the room
                    room = 4                            #Set which room bot is in
                    Search_room(room, fire_extinguished, inside_room, hall, dog_config)     #Search the room the bot is in
                    room_checked[room - 1] = True           #Mark off this room as checked for the candle
                    inside_room = False                 #Mark the bot is back in the hall
                    hall = 9                            #Set the hall to 9
                    entrance = 5                        #Set the entrance to be the top entrance of room 4
                    position = 4                        #Set the new position                   
                if (line_detected):                     #If the entrance to room 4 was found
                    Align_on_line()                     #Align on the line
                    line_detected = False               #Reset the line sensors
                    inside_room = True                  #Tell the bot it's outside the room
                    room = 4                            #Set which room bot is in
                    Search_room(room, fire_extinguished, inside_room, hall, dog_config)     #Search the room the bot is in
                    room_checked[room - 1] = True           #Mark off this room as checked for the candle
                    inside_room = False                 #Mark the bot is back in the hall
                    hall = 6                            #Set the hall to 6
                    entrance = 4                        #Set the entrance to be the lower entrance of room 4
                    position = 6                        #Set the new position
                    
if (test == 17):
    position = 4
    hall = 4
    if (position == 4):                     #If the robot is at the top entrance of room 3
        Transition_forward(TRANS*3, SPEED)  #Transition forward far enough to go off line
        line_detected, dog_config = Right_wall_follow_until_turn(inside_room, hall, dog_config)  #Follow until the top wall
        if (dog_config == 2):               #If there is a dog in hall 4
            Turn_left(90,TURNSPEED)         #Turn left 90 degrees
            Move_forward_while_checking(10,SPEED,line_detected)  #Move forward looking for a wall
            Turn_left(90,TURNSPEED)         #Turn left 90 degrees
            hall = 7                        #Set the hall to 7
            position = 5                    #Set the new position
        else:
            Right_wall_follow_for_distance(46)  #Follow the right wall for 46 cm
            Turn_left(90,TURNSPEED)             #Turn right 90 degrees
            line_detected = Move_forward_while_checking(10,SPEED,line_detected)  #Move forward looking for a line or wall
            if (line_detected):                     #If the top entrance to room 4 was found
                Align_on_line()                     #Align on the line
                line_detected = False               #Reset the line sensors
                inside_room = True                  #Tell the bot it's outside the room
                room = 4                            #Set which room bot is in
                Search_room(room, fire_extinguished, inside_room, hall, dog_config)     #Search the room the bot is in
                room_checked[room - 1] = True       #Mark off this room as checked for the candle
                inside_room = False                 #Mark the bot is back in the hall
                hall = 7                            #Set the hall to 7
                entrance = 5                        #Set the entrance to be the upper entrance of room 4
                position = 5                        #Set the new position
            else:                                   #If room 4 entrance is on bottom
                Turn_left(90,TURNSPEED)             #Turn left 90 degrees
                hall = 7                            #Set the hall to 7
                position = 5                        #Set the new position
                position = 5                        #Set the new position

if (test == 18):
    position = 5
    room_checked[3] = True   #Change this to test if you want to test room 4 being checked already or not
    if (position == 5):                     #If the robot is at the top entrance of room 4
        if (room_checked[3] and (not dog_config == 3)):     #If the robot began in room 4 and there isn't a dog in the top hall
            Transition_forward(TRANS*3, SPEED)      #Move off the line
            Move_forward_while_checking(30,SPEED,line_detected)  #Move forward looking for a wall
            Turn_until_aligned()                    #Turn until aligned with the wall
            Move_forward_until_dist(WALLDIST,SPEED/2) #Make sure the robot is still WALLDIST away
            Turn_left(90,TURNSPEED)                 #Turn left 90 degrees
            line_detected, dog_config = Right_wall_follow_until_door(inside_room, hall, dog_config)  #Follow until the next room
            if (line_detected):                     #If the top entrance to room 4 was found
                Align_on_line()                     #Align on the line
                line_detected = False               #Reset the line sensors
                inside_room = True                  #Tell the bot it's outside the room
                room = 1                            #Set which room bot is in
                Search_room(room, fire_extinguished, inside_room, hall, dog_config)     #Search the room the bot is in
                room_checked[room - 1] = True       #Mark off this room as checked for the candle
                inside_room = False                 #Mark the bot is back in the hallMove_forward_until_dist(WALLDIST,SPEED/2) #Make sure the robot is still WALLDIST away
                Turn_until_aligned()                    #Turn until aligned with the wall
                Move_forward_until_dist(WALLDIST,SPEED/2) #Make sure the robot is still WALLDIST away
                hall = 1                            #Set the hall to 1
                position = 1                        #Set the new position
            else:
                Turn_left(90,TURNSPEED)             #Turn left to face room 4 again
                line_detected = Move_forward_while_checking(10,SPEED,line_detected)  #Move forward looking for a wall or lione
                if (line_detected):                     #If the top entrance to room 4 was found
                    Align_on_line()                     #Align on the line
                    line_detected = False               #Reset the line sensors
                    Enter_and_exit_room()               #Realign robot at the door
                else:
                    Right_wall_follow_until_door(inside_room, hall, dog_config)  #Follow until door
                    Align_on_line()                     #Align on the line
                    Enter_and_exit_room()               #Realign robot at the door
        else:
            Transition_forward(TRANS*3, SPEED)  #Transition forward far enough to go off line
            line_detected, dog_config = Right_wall_follow_until_door(inside_room, hall, dog_config)  #Follow until the next room
            if (line_detected):                     #If the entrance to room 4 was found
                Align_on_line()                     #Align on the line
                line_detected = False               #Reset the line sensors
                inside_room = True                  #Tell the bot it's outside the room
                room = 4                            #Set which room bot is in
                Search_room(room, fire_extinguished, inside_room, hall, dog_config)     #Search the room the bot is in
                room_checked[room - 1] = True       #Mark off this room as checked for the candle
                inside_room = False                 #Mark the bot is back in the hall
                hall = 6                            #Set the hall to 6
                entrance = 4                        #Set the entrance to be the lower entrance of room 4
                position = 6                        #Set the new position
            
if (test == 19):
    position = 6
    if (position == 6):                             #If the robot is at the bottom of room 4
        Transition_forward(TRANS,SPEED)
        Move_forward_while_checking(30,SPEED,line_detected)  #Move forward looking for a wall
        Move_forward_until_dist(WALLDIST,SPEED/2) #Make sure the robot is still WALLDIST away
        Turn_until_aligned()                    #Turn until aligned with the wall
        Move_forward_until_dist(WALLDIST,SPEED/2) #Make sure the robot is still WALLDIST away
        Turn_right(100,TURNSPEED)
        Transition_forward(TRANS,SPEED)
        Move_forward(11,SPEED)                  #Move forward 11 cm
        Turn_right(80,TURNSPEED)
        Transition_forward(TRANS,SPEED)
        Move_forward(6,SPEED)
        Right_wall_follow_for_distance(30)
        Turn_left(90,TURNSPEED)
        Move_forward_until_dist(WALLDIST,SPEED/2) #Make sure the robot is still WALLDIST away
        Turn_until_aligned()                    #Turn until aligned with the wall
        Move_forward_until_dist(WALLDIST,SPEED/2) #Make sure the robot is still WALLDIST away
        Turn_left(90,TURNSPEED)
        line_detected, dog_config = Right_wall_follow_until_door(inside_room, hall, dog_config)  #Follow until the next room
        if (line_detected):                     #If the top entrance to room 4 was found
            Align_on_line()                     #Align on the line
            line_detected = False               #Reset the line sensors
            inside_room = True                  #Tell the bot it's outside the room
            room = 1                            #Set which room bot is in
            Search_room(room, fire_extinguished, inside_room, hall, dog_config)     #Search the room the bot is in
            room_checked[room - 1] = True       #Mark off this room as checked for the candle
            inside_room = False                 #Mark the bot is back in the hall
            hall = 1                            #Set the hall to 1
            position = 1                        #Set the new position

if (test == 20):    #UV Tron test
    fire_detected = 0
    for x in range(0,10000):
        fire_detected += GPIO.input(12)
    if (fire_detected < 10000):
        print ("Fire Detected: ", fire_detected)
    else:
        print ("Fire not detected")
    
if (test == 21):    #MOSFET test
    GPIO.output(16, GPIO.HIGH)
    time.sleep(1)
    GPIO.output(16, GPIO.LOW)
    
if (test == 22):
    room = 1
    fire_extinguished = False
    inside_room = False
    hall = 1
    dog_config = 0
    Search_room(room, fire_extinguished, inside_room, hall, dog_config)
    
print ("Done.")
while(True):
    GPIO.cleanup()
    Stop(100)
