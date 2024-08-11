import os
from flask import Flask, request, jsonify
import RPi.GPIO as GPIO
from gpiozero import Servo
from gpiozero.pins.pigpio import PiGPIOFactory
import serial
import time
import threading

app = Flask(__name__)
# Chan Step
ENABLE = 16 
DIR = 20  
STEP = 21  
CW = 1  # Quay thuan
CCW = 0  # Quay nguoc
SPR = 200  # Steps per revolution 

# Position tracking
current_position = 0  # Current position in degrees
MIN_POSITION = 0     
MAX_POSITION = 360   

# Chan GPIO cho led
LED_PIN_1 = 13
LED_PIN_2 = 19
LED_PIN_3 = 26
LED_PIN_4 = 25
FAN_PIN_4 = 6
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(LED_PIN_1, GPIO.OUT)
GPIO.setup(LED_PIN_2, GPIO.OUT)
GPIO.setup(LED_PIN_3, GPIO.OUT)
GPIO.setup(LED_PIN_4, GPIO.OUT)
GPIO.setup(FAN_PIN_4, GPIO.OUT)
GPIO.setup(DIR, GPIO.OUT)
GPIO.setup(STEP, GPIO.OUT)
GPIO.setup(ENABLE, GPIO.OUT)

# Ham Step
def move_motor(degrees, direction):
    global current_position

    if direction == CW:
        target_position = current_position + degrees
    elif direction == CCW:
        target_position = current_position - degrees
    else:
        return

    # Check if target position is within bounds
    if target_position < MIN_POSITION or target_position > MAX_POSITION:
        return  # Out of bounds, do nothing

    # Calculate steps needed for given degrees
    if degrees == 90:
        steps = int(SPR * 0.25)  # 25% of one revolution
    elif degrees == 180:
        steps = int(SPR * 0.5)  # 50% of one revolution
    elif degrees == 360:
        steps = SPR  # 100% of one revolution
    # elif degrees == 720:
    #    steps = SPR * 2  # 200% of one revolution
    else:
        return

    GPIO.output(ENABLE, GPIO.LOW)  # Enable the motor
    GPIO.output(DIR, direction)  # Set direction

    for _ in range(steps):
        GPIO.output(STEP, GPIO.HIGH)
        time.sleep(0.001)
        GPIO.output(STEP, GPIO.LOW)
        time.sleep(0.001)

    # Update current position
    current_position = target_position
    GPIO.output(ENABLE, GPIO.LOW)  # Keep motor enabled to hold position

# Chan GPIO cho servo
servo1_pin = 17  #
servo2_pin = 18
factory = PiGPIOFactory()
servo1 = Servo(servo1_pin, pin_factory=factory)  # Updated to use servo1_pin
servo2 = Servo(servo2_pin, pin_factory=factory)

def set_angle(servo1, angle):
    # Convert the angle to a value between -1 and 1
    # Assumes 0 degrees = -1, 90 degrees = 0, and 180 degrees = 1
    value = (angle / 60) - 1
    value = max(min(value, 1), -1)
    servo1.value = value
    servo2.value = value

set_angle(servo1, 120)  # Updated to use servo1
set_angle(servo2, 0)

# Cau hinh cho arduino
ser = serial.Serial('/dev/serial0', 9600, timeout=1)
time.sleep(2)

def send_command(command):
    ser.write((command + '\n').encode())
    time.sleep(2)  # Give the Arduino some time to respond
    return ser.readline().decode('utf-8').strip()
# Ham nhan du lieu lien tuc tu Arduino
def read_from_arduino():
    while True:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8').rstrip()
        #    print("Received:", line)  
        time.sleep(1)
# Tao luong de nhan du lieu lien tuc tu Arduino
thread = threading.Thread(target=read_from_arduino)
thread.daemon = True
thread.start()
# GAS
def get_gas_sensor_data():
 
    gas_data = ser.readline().decode().strip()
    return int(gas_data)

@app.route('/webhook', methods=['POST'])
def webhook():
    req = request.get_json(silent=True, force=True)
    intent = req.get('queryResult').get('intent').get('displayName')
    
    if intent == 'ioutput':
        led = req.get('queryResult').get('parameters').get('led')
        action = req.get('queryResult').get('parameters').get('action')
        
        if led == 'led1':
            if action == 'on':
                GPIO.output(LED_PIN_1, GPIO.HIGH)     
                return jsonify({'fulfillmentText': 'LED1 is turned on'})         
            elif action == 'off':
                GPIO.output(LED_PIN_1, GPIO.LOW)
                return jsonify({'fulfillmentText': 'LED1 is turned off'})
        
        elif led == 'led2':
            if action == 'on':
                GPIO.output(LED_PIN_2, GPIO.HIGH)
                return jsonify({'fulfillmentText': 'LED2 is turned on'})           
            elif action == 'off':
                GPIO.output(LED_PIN_2, GPIO.LOW)
                return jsonify({'fulfillmentText': 'LED2 is turned off'})
            
        elif led == 'led3':
            if action == 'on':
                GPIO.output(LED_PIN_3, GPIO.HIGH)
                return jsonify({'fulfillmentText': 'LED3 is turned on'})
            elif action == 'off':
                GPIO.output(LED_PIN_3, GPIO.LOW)
                return jsonify({'fulfillmentText': 'LED3 is turned off'})  
        
        elif led == 'led4':
            if action == 'on':
                GPIO.output(LED_PIN_4, GPIO.HIGH)
                return jsonify({'fulfillmentText': 'LED4 is turned on'})
            elif action == 'off':
                GPIO.output(LED_PIN_4, GPIO.LOW)
                return jsonify({'fulfillmentText': 'LED4 is turned off'})  
        
        elif led == 'fan':
            if action == 'on':
                GPIO.output(FAN_PIN_4, GPIO.HIGH)
                return jsonify({'fulfillmentText': 'FAN is turned on'})
            elif action == 'off':
                GPIO.output(FAN_PIN_4, GPIO.LOW)
                return jsonify({'fulfillmentText': 'FAN is turned off'})

        elif led == 'all':
            if action == 'on':
                GPIO.output(LED_PIN_1, GPIO.HIGH)
                GPIO.output(LED_PIN_2, GPIO.HIGH)
                GPIO.output(LED_PIN_3, GPIO.HIGH)
                return jsonify({'fulfillmentText': 'All LEDs are turned on'})
            elif action == 'off':
                GPIO.output(LED_PIN_1, GPIO.LOW)
                GPIO.output(LED_PIN_2, GPIO.LOW)
                GPIO.output(LED_PIN_3, GPIO.LOW)
                return jsonify({'fulfillmentText': 'All LEDs are turned off'})
        
        return jsonify({'fulfillmentText': 'Unknown command for the specified LED'})
    
    elif intent == 'iservo':
        action = req.get('queryResult').get('parameters').get('action')
        servo_action = req.get('queryResult').get('parameters').get('servo_action')
        
        if action == 'on':
            set_angle(servo1, 0)
            set_angle(servo2, 120)
            time.sleep(2)
            return jsonify({'fulfillmentText': 'Opened the door!'})
        elif action == 'off':
            set_angle(servo1, 120)
            set_angle(servo2, 0)
            time.sleep(2)
            return jsonify({'fulfillmentText': 'Closed the door!'})
        else:
            return jsonify({'fulfillmentText': 'Unknown command!'})
    elif intent == 'itemp_hum':
        temp_hum = req.get('queryResult').get('parameters').get('temp_hum')
        doC_doF = req.get('queryResult').get('parameters').get('doC_doF')
    
        if temp_hum == 'temperature':
            if doC_doF == 'doC':
                ser.write("GET_TEMP_C\n".encode())
                sensor_data = ser.readline().decode('utf-8').strip()
                return jsonify({'fulfillmentText': f"Temperature in Celsius: {sensor_data} C"})
           
            elif doC_doF == 'doF':
                ser.write("GET_TEMP_F\n".encode())
                sensor_data = ser.readline().decode('utf-8').strip()
                return jsonify({'fulfillmentText': f"Sensor data:\n{sensor_data} F"})
            
            else:
                return jsonify({'fulfillmentText': 'Invalid temperature unit command!'})

        elif temp_hum == 'humidity':
            ser.write("GET_HUM\n".encode())
            sensor_data = ser.readline().decode('utf-8').strip()
            return jsonify({'fulfillmentText': f"Sensor data:\n{sensor_data} %"})

        else:
            return jsonify({'fulfillmentText': 'Request not recognized!'})

    elif intent == 'iMQ2_gas':
        check_gas = req.get('queryResult').get('parameters').get('check_gas')
        
        if check_gas == 'gas':
            ser.write("GET_GAS\n".encode())
            time.sleep(2) 
            
            sensor_data = ""
            for _ in range(10):
                line = ser.readline().decode('utf-8').strip()
                sensor_data += f"{line} PPM, " 

            return jsonify({'fulfillmentText': f"Gas concentration: {sensor_data} PPM"})
        else:
            return jsonify({'fulfillmentText': 'Request not recognized!'})
  

    elif intent == 'istep':
        action = req.get('queryResult').get('parameters').get('action')
        curtain_angle = req.get('queryResult').get('parameters').get('curtain_angle')
       
        if curtain_angle == 'curtain full':
            if action == 'on':
                move_motor(360, CW)
                return jsonify({'fulfillmentText': 'Opened the curtain full!'})         
            elif action == 'off':
                move_motor(360, CCW)
                return jsonify({'fulfillmentText': 'Closed the curtain full!'})
     
        elif curtain_angle == 'curtain 1/2':
            if action == 'on':
                move_motor(180, CW)
                return jsonify({'fulfillmentText': 'Opened the curtain 1/2!'})         
            elif action == 'off':
                move_motor(180, CCW)
                return jsonify({'fulfillmentText': 'Closed the curtain 1/2!'})
        
        else:
            return jsonify({'fulfillmentText': 'Unknown intent'})

    else:
        return jsonify({'fulfillmentText': 'Unknown intent'})

# if __name__ == '__main__':
#    port = int(os.getenv('PORT',5000))
#    print("Staring app on port %d" %(port))
#    app.run(debug = True, port=port, host='0.0.0.0')

if __name__ == '__main__':
    try:
        port = int(os.getenv('PORT', 6066))
        print("Starting app on port %d" % port)
        app.run(debug=True, port=port, host='0.0.0.0')
    except KeyboardInterrupt:
        servo1_pin.detach()
        servo2_pin.detach()
        GPIO.cleanup()
