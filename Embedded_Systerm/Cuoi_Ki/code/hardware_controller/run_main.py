#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import serial
import time
import RPi.GPIO as GPIO
import spidev
import smbus
import math



speed_linear = 0.2
speed_angular = 0.6
state_count = 0
state_mode = 0

#Function GPIO INTERRUPT
def speed_linear_control_B1(channel):
    global speed_linear 
    speed_linear= speed_linear + 0.1
    speed_linear =round(speed_linear,2)
    display_velocity(speed_linear*3.6, 5)
    display_velocity(speed_angular*3.6, 1)
    print('speed_linear: ', speed_linear)
    
def speed_linear_control_B2(channel):
    global speed_linear 
    speed_linear= speed_linear - 0.1
    if speed_linear < 0.0:
        speed_linear = 0.0
    speed_linear =round(speed_linear,2)
    display_velocity(speed_linear*3.6, 5)
    display_velocity(speed_angular*3.6, 1)
    print('speed_linear: ', speed_linear)

def speed_linear_control_B3(channel):
    global speed_angular 
    speed_angular =  speed_angular + 0.1
    speed_angular =round(speed_angular,2)
    display_velocity(speed_linear*3.6, 5)
    display_velocity(speed_angular*3.6, 1)
    print('speed_angular: ', speed_angular)

def speed_linear_control_B4(channel):
    global speed_angular 
    speed_angular =  speed_angular - 0.1
    if speed_angular < 0.0:
        speed_angular = 0.0
    speed_angular =round(speed_angular,2)
    display_velocity(speed_linear*3.6, 5)
    display_velocity(speed_angular*3.6, 1)
    print('speed_angular: ', speed_angular)

# control state 
def control_state (channel):
    global state_mode
    global state_count
    state_mode += 1
    if state_mode == 1 or state_mode == 2:
        display_velocity(speed_linear*3.6, 5)
        display_velocity(speed_angular*3.6, 1)
    if state_mode > 2:
        state_count = 0
        state_mode = 0

# Function to send data to MAX7219
def send_data_spi0(reg, data):
    spi0.xfer2([reg, data])

# Function to send data to MAX7219
def send_data_spi1(reg, data):
    spi1.xfer2([reg, data])

# Calculate roll, pitch, yaw
def calculate_angles(accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z):
    accel_x -= accel_offset_x
    accel_y -= accel_offset_y
    accel_z -= accel_offset_z

    gyro_x -= gyro_offset_x
    gyro_y -= gyro_offset_y
    gyro_z -= gyro_offset_z

    dt = 0.01  # Time step (adjust as needed)
    alpha = 0.98  # Complementary filter constant (adjust as needed)

    # Calculate roll angle
    roll_acc = math.atan2(accel_y, accel_z) * 180.0 / math.pi
    roll_gyro = roll_acc + gyro_x * dt
    roll = alpha * roll_gyro + (1 - alpha) * roll_acc

    # Calculate pitch angle
    pitch_acc = math.atan2(-accel_x, math.sqrt(accel_y * accel_y + accel_z * accel_z)) * 180.0 / math.pi
    pitch_gyro = pitch_acc + gyro_y * dt
    pitch = alpha * pitch_gyro + (1 - alpha) * pitch_acc

    # Calculate yaw angle
    yaw = gyro_z * dt

    return roll, pitch, yaw

# Read MPU6050 data
def read_data(reg):
    high = bus.read_byte_data(MPU6050_ADDR, reg)
    low = bus.read_byte_data(MPU6050_ADDR, reg + 1)
    value = (high << 8) | low
    if value > 32767:
        value -= 65536
    return value

def display_velocity(num, position):
    a = abs(num) if num >= 0 else abs(num) * -1
    integerPart = abs(int(num))
    fractionalPart = int((a - integerPart) * 100)
    number = integerPart * 100 + fractionalPart

    # count the number of digits
    n = number
    count = position
    while n // 10:
        count += 1
        n = n // 10

    if number < 100:
        send_data_spi1(position+2, 0x7E | 0x80)
        if number <= 0 :
            send_data_spi1(position, 0x7E)
            send_data_spi1(position+1, 0x7E)

    # display number
    for i in range(position-1, count):
        if i == position+1:
            send_data_spi1(i+1, (digit[number%10]) | 0x80)  # turn on dot segment
        else:
            send_data_spi1(i+1, digit[number%10])
        number = number // 10

if __name__ == '__main__':
    rospy.init_node('Send_Msgs')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    sdata = serial.Serial('/dev/ttyACM0', 9600, timeout=1.0)
    sdata.reset_input_buffer()
    print("Arduino Connected")
    move = Twist()
    move.linear.x = 0.0
    move.angular.z = 0.0
    s1=0
    s2=0
    # ---------------------- SPI0 -------------------------------
    # MAX7219 registers
    MAX7219_REG_NOOP = 0x00
    MAX7219_REG_DIGIT0 = 0x01
    MAX7219_REG_DIGIT1 = 0x02
    MAX7219_REG_DIGIT2 = 0x03
    MAX7219_REG_DIGIT3 = 0x04
    MAX7219_REG_DIGIT4 = 0x05
    MAX7219_REG_DIGIT5 = 0x06
    MAX7219_REG_DIGIT6 = 0x07
    MAX7219_REG_DIGIT7 = 0x08
    MAX7219_REG_DECODEMODE = 0x09
    MAX7219_REG_INTENSITY = 0x0A
    MAX7219_REG_SCANLIMIT = 0x0B
    MAX7219_REG_SHUTDOWN = 0x0C
    MAX7219_REG_DISPLAYTEST = 0x0F

    # Initialize SPI
    spi0 = spidev.SpiDev()
    spi1 = spidev.SpiDev()

    spi0.open(0, 0)
    spi1.open(1, 2)

    spi0.mode = 0                # Set the SPI mode (0 or 3)
    spi1.mode = 0
    spi0.max_speed_hz = 1000000
    spi1.max_speed_hz = 1000000

    # Initialize MAX7219
    send_data_spi0(MAX7219_REG_SCANLIMIT, 7)  # Show all 8 digits
    send_data_spi0(MAX7219_REG_DECODEMODE, 0)  # No BCD decoding
    send_data_spi0(MAX7219_REG_DISPLAYTEST, 0)  # Disable test mode
    send_data_spi0(MAX7219_REG_SHUTDOWN, 1)  # Wake up from shutdown
    send_data_spi0(MAX7219_REG_INTENSITY, 15)  # Set brightness (0-15)

    # Initialize MAX7219
    send_data_spi1(MAX7219_REG_SCANLIMIT, 7)  # Show all 8 digits
    send_data_spi1(MAX7219_REG_DECODEMODE, 0)  # No BCD decoding
    send_data_spi1(MAX7219_REG_DISPLAYTEST, 0)  # Disable test mode
    send_data_spi1(MAX7219_REG_SHUTDOWN, 1)  # Wake up from shutdown
    send_data_spi1(MAX7219_REG_INTENSITY, 15)  # Set brightness (0-15)

    stop = [
        0b01111100,
        0b01100110,
        0b01100110,
        0b01111100,
        0b01100000,
        0b01100000,
        0b01100000,
        0b01100000
    ]

    # Arrow pattern
    arrow_rotate_left = [
        0b00111100,
        0b01000010,
        0b10000001,
        0b10000001,
        0b10010001,
        0b01010001,
        0b00110010,
        0b11110000
    ]

    arrow_rotate_right = [
        0b00111100,
        0b01000010,
        0b10000001,
        0b10000001,
        0b10001001,
        0b10001010,
        0b01001100,
        0b00001111
    ]

    arrow_go_front = [0x18,0x3C,0x5A,0x99,0x18,0x18,0x18,0x18]
    arrow_go_back = [0x18,0x18,0x18,0x18,0x99,0x5A,0x3C,0x18]
    digit = [0x7E,0x30,0x6D,0x79,0x33,0x5B,0x5F,0x70,0x7F,0x7B] # Hex Digit

    # Clear the display
    for i in range(8):
        send_data_spi0(i + 1, 0)

    # Clear the display
    for i in range(8):
        send_data_spi1(i + 1, 0)

    # ---------------------- GPIO INTERRUPT -------------------------------

    # Thiết lập chân GPIO và chế độ ngắt
    GPIO.setmode(GPIO.BCM)
    interrupt_pin =[5,6,12,13,17]

    for i in range (0, len(interrupt_pin)):
        GPIO.setup(interrupt_pin[i], GPIO.IN, pull_up_down=GPIO.PUD_UP)  # Chân GPIO đầu ra

    # Thiết lập ngắt trên chân GPIO
    GPIO.add_event_detect(5, GPIO.RISING,  callback=speed_linear_control_B3, bouncetime =200)
    GPIO.add_event_detect(12, GPIO.RISING,  callback=speed_linear_control_B2, bouncetime =200)
    GPIO.add_event_detect(6, GPIO.RISING,  callback=speed_linear_control_B1, bouncetime =200)
    GPIO.add_event_detect(13, GPIO.RISING,  callback=speed_linear_control_B4, bouncetime =200)
    GPIO.add_event_detect(17, GPIO.RISING,  callback=control_state, bouncetime =200)

    # ------------------------ MPU SETUP ---------------------------------
    # MPU6050 registers
    MPU6050_ADDR = 0x68
    MPU6050_REG_ACCEL_X = 0x3B
    MPU6050_REG_ACCEL_Y = 0x3D
    MPU6050_REG_ACCEL_Z = 0x3F
    MPU6050_REG_GYRO_X = 0x43
    MPU6050_REG_GYRO_Y = 0x45
    MPU6050_REG_GYRO_Z = 0x47

    # Initialize I2C bus
    bus = smbus.SMBus(1)  # Use I2C bus 1

    # MPU6050 setup
    bus.write_byte_data(MPU6050_ADDR, 0x6B, 0x00)  # Power on MPU6050

    # MPU6050 calibration values (adjust these for your specific MPU6050)
    accel_offset_x = 0
    accel_offset_y = 0
    accel_offset_z = 0

    gyro_offset_x = 0
    gyro_offset_y = 0
    gyro_offset_z = 0
   
    rate = rospy.Rate(2)
    # rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        move = Twist()
        count = 0
        state_error = 0

        print("state mode : \n", state_mode)

        if state_mode == 0 and state_count == 0 :
            state_count = 1
            # Clear the display
            for i in range(8):
                send_data_spi0(i + 1, 0)

            # Clear the display
            for i in range(8):
                send_data_spi1(i + 1, 0)

        while state_mode == 1 :
            if sdata.in_waiting > 0 :
                mydata = sdata.readline().decode('utf-8').rstrip()
                print(mydata)
                if state_error < 2 or mydata=="":
                    state_error += 1
                    s1 = 519
                    s2 = 514
                else:

                    buffer = mydata.split(",")
                    s1 = int(buffer[0])
                    s2 = int(buffer[1])
                    print("s1: {:d}".format(s1))
                    print("s2: {:d}".format(s2))
                    print(mydata) 
                   
            # Stop
            if (s1 >= 500 and s1 <= 530) and (s2 >= 500 and s2 <= 530) :
                move.linear.x = 0.0
                move.angular.z = 0.0
                pub.publish(move)
                for i in range(8):
                    send_data_spi0(i + 1, stop[i])
            # Move Front 
            if (s1 >= 500 and s1 <= 530) and (s2 >= 0 and s2 < 500)  :
                move.linear.x = speed_linear
                move.angular.z = 0.0
                pub.publish(move)
                # Display arrow on the LED matrix
                for i in range(8):
                    send_data_spi0(i + 1, arrow_go_front[i])
                
            # Move Back
            if (s1 >= 500 and s1 <= 530) and (s2 <=1023 and s2>530)  :
                move.linear.x = -speed_linear
                move.angular.z = 0.0
                pub.publish(move)
                # Display arrow on the LED matrix
                for i in range(8):
                    send_data_spi0(i + 1, arrow_go_back[i])
            
            # Move angular left 
            if (s1 >= 0 and s1 <= 500) and (s2 >= 0 and s2 <= 1023)  :
                move.linear.x = 0.0
                move.angular.z = speed_angular
                pub.publish(move)
                for i in range(8):
                    send_data_spi0(i + 1, arrow_rotate_left[i])
                
            # Move angular right
            if (s1 >= 530 and s1 <= 1023) and (s2 >= 0 and s2 <= 1023)  :
                move.linear.x = 0.0
                move.angular.z = -speed_angular
                pub.publish(move)
                for i in range(8):
                    send_data_spi0(i + 1, arrow_rotate_right[i])
            time.sleep(0.07) 
            # print('rate')q    


        while state_mode == 2 :
            accel_x = read_data(MPU6050_REG_ACCEL_X)
            accel_y = read_data(MPU6050_REG_ACCEL_Y)
            accel_z = read_data(MPU6050_REG_ACCEL_Z)
            gyro_x = read_data(MPU6050_REG_GYRO_X)
            gyro_y = read_data(MPU6050_REG_GYRO_Y)
            gyro_z = read_data(MPU6050_REG_GYRO_Z)

            roll, pitch, yaw = calculate_angles(accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z)

            if roll >90 :
                roll=90
            if roll<-90:
                roll=-90
            if pitch >90 :
                pitch=90
            if pitch<-90:
                pitch=-90
            if yaw >90 :
                yaw=90
            if yaw<-90:
                yaw=-90
                    
            # Stop
            if (pitch >= -40 and pitch <= 40) and (roll >= -40 and roll <= 40) :
                move.linear.x = 0.0
                move.angular.z = 0.0
                pub.publish(move)
                for i in range(8):
                    send_data_spi0(i + 1, stop[i])
            ## right
            if(roll>40 and roll<=90) and(pitch>=-40 and pitch <=40):
                move.linear.x = 0.0
                move.angular.z = -speed_angular
                pub.publish(move)
                for i in range(8):
                    send_data_spi0(i + 1, arrow_rotate_right[i])

            ##left
            if(roll>=-90 and roll<-40) and(pitch>=-40 and pitch <=40):
                move.linear.x = 0.0
                move.angular.z = speed_angular
                pub.publish(move)
                for i in range(8):
                    send_data_spi0(i + 1, arrow_rotate_left[i])

            
            ##front
            if(roll>=-90 and roll<=90) and(pitch>40 and pitch <=90):
                move.linear.x = speed_linear
                move.angular.z = 0.0
                pub.publish(move)
                for i in range(8):
                    send_data_spi0(i + 1, arrow_go_front[i])

            ##back
            if(roll>=-90 and roll<=90) and(pitch>=-90 and pitch <=-40):
                move.linear.x = -speed_linear
                move.angular.z = 0.0
                pub.publish(move)
                for i in range(8):
                    send_data_spi0(i + 1, arrow_go_back[i])

            # Print the sensor data
            print(f"Roll: {roll:.2f} degrees")
            print(f"Pitch: {pitch:.2f} degrees")
            time.sleep(0.07) 
            



    
       
