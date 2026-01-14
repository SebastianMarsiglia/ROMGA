import time
import serial
import struct

# Definiciones
START_FRAME = 0xABCD
TIME_SEND = 0.1  # Tiempo en segundos (100 ms)

# Configuración UART en la Raspberry Pi (puerto y velocidad)
ser = serial.Serial('/dev/serial0', 115200, timeout=1)

def send_command(steer, speed):
    # Cálculo del checksum
    checksum = (START_FRAME ^ steer ^ speed) & 0xFFFF
    
    # Empaquetado de los datos (2 bytes cada uno)
    data = struct.pack('<HhhH', START_FRAME, steer, speed, checksum)
    
    # Envío de los datos por UART
    ser.write(data)

# Inicialización de variables
iSpeed = 0
MAX_SPEED = 1000
iSteer = 0
MAX_STEER = 1000
iTimeSend = time.time()

while True:
    # Limitar las variables a sus máximos
    iSpeed = max(min(iSpeed, MAX_SPEED), -MAX_SPEED)
    iSteer = max(min(iSteer, MAX_STEER), -MAX_STEER)

    # Enviar comandos periódicamente
    if time.time() > iTimeSend:
        iTimeSend = time.time() + TIME_SEND
        send_command(iSteer, iSpeed)
    
    # Actualizar valores
    iSpeed = 100                          #Se aplica directo a ambos motores, positivo para adelante, negativo para reversa
    iSteer = -200                         #Se suma la mitad al motor derecho, se resta la mitad al motor izquierdo.
    
    time.sleep(0.01)  # Pequeño delay para evitar alto uso de CPU