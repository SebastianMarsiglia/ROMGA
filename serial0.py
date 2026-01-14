import time
import serial

ser = serial.Serial('/dev/ttyS0', 115200, timeout=2)  # Configuración del puerto serial de la Raspberry Pi a 115200 baudios con un timeout de 1 segundo.
# Inicializa el puerto y limpia el buffer
ser.reset_input_buffer()
