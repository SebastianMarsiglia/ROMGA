# robot_control.py

import time
import struct
import serial
import os
import RPi.GPIO as GPIO
from picamera2 import Picamera2
from flask import Response
import cv2

START_FRAME = 0xABCD
TIME_SEND = 0.05
MAX_STEER = 100
MAX_SPEED = 100
manual_mode=False
PIN = 18
duracion = 0.5

GPIO.setmode(GPIO.BCM)
GPIO.setup(PIN, GPIO.OUT)
GPIO.output(PIN, GPIO.LOW)

# Estado compartido
steerjoy = 0
speedjoy = 0
batVoltage = 0
voltajereal = 0
speedR_meas = 0
speedL_meas = 0
cmd1 = 0
cmd2 = 0
last_send_time = 0
picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"size": (640, 360)})
picam2.configure(config)
picam2.set_controls({"AfMode": 2})
picam2.start()
time.sleep(1)
# Inicializar UART
try:
    print("Encendiendo GPIO al inicio...")
    #GPIO.output(PIN, GPIO.HIGH)
    time.sleep(duracion)
   # GPIO.output(PIN, GPIO.LOW)

    ser = serial.Serial('/dev/ttyAMA0', 115200, timeout=2)
    ser.reset_input_buffer()
except serial.SerialException as e:
    print(f"Error UART: {e}")
    exit(1)

def speak(text):
    speed = 1.2
    pitch = 100
    os.system(f'pico2wave -l es-ES -w output.wav "{text}" && sox output.wav adjusted_output.wav tempo {speed} pitch {pitch} && aplay adjusted_output.wav && rm output.wav adjusted_output.wav')

def calculate_checksum(start, steer, speed):
    return (start ^ steer ^ speed) & 0xFFFF

def send_command(steer, speed):
    steer = int(max(-MAX_STEER, min(steer, MAX_STEER)))
    speed = int(max(-MAX_SPEED, min(speed, MAX_SPEED)))
    checksum = calculate_checksum(START_FRAME, steer, speed)
    command = struct.pack('<HhhH', START_FRAME, steer, speed, checksum)

    try:
        ser.write(command)
    except serial.SerialException as e:
        print(f"Error enviando comando: {e}")

def parse_feedback(data):
    global batVoltage, speedR_meas, speedL_meas, cmd1, cmd2
    feedback = struct.unpack('<HhhhhhhHH', data)
    start, cmd1, cmd2, speedR_meas, speedL_meas, batVoltage, boardTemp, cmdLed, checksum = feedback
    calc_checksum = (start ^ cmd1 ^ cmd2 ^ speedR_meas ^ speedL_meas ^ batVoltage ^ boardTemp ^ cmdLed) & 0xFFFF
    if start == START_FRAME and calc_checksum == checksum:
        print(f"VB: {batVoltage/100:.1f}V, RPMD: {speedR_meas}, RPMI: {speedL_meas}")

def receive_feedback():
    if ser.is_open and ser.in_waiting >= 18:
        data = ser.read(18)
        parse_feedback(data)

def update_control(steer, speed):
    global steerjoy, speedjoy
    steerjoy = steer
    speedjoy = speed
def video_feed():
    def generate():
        while True:
            frame = picam2.capture_array()
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            ret, buffer = cv2.imencode('.jpg', frame)
            yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

def control_loop(socketio_emit_fn):
    global steerjoy, speedjoy, last_send_time, batVoltage, voltajereal

    while manual_mode:
        #receive_feedback()
        if ser.is_open:
            ser.reset_input_buffer()

        current_time = time.time()
        if current_time - last_send_time >= TIME_SEND:
            send_command(steerjoy, speedjoy)
            last_send_time = current_time

        voltajereal = batVoltage / 100
        socketio_emit_fn({'batVoltage': voltajereal})

        time.sleep(0.01)

def shutdown():
    #GPIO.output(PIN, GPIO.HIGH)
    time.sleep(duracion)
    #GPIO.output(PIN, GPIO.LOW)
    #GPIO.cleanup()
    if ser.is_open:
        ser.close()
