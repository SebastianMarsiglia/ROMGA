from gevent import monkey    #Para gevent
monkey.patch_all()              

from flask import Flask, render_template, Response
from flask_socketio import SocketIO
from picamera2 import Picamera2
import cv2
import time
import serial
import struct

START_FRAME = 0xABCD  # Definición del valor inicial (start frame) que indica el comienzo de un paquete de datos para garantizar una comunicación serial confiable.
TIME_SEND = 0.05    # Intervalo de tiempo (en segundos) entre el envío de comandos (50 ms).
MAX_STEER = 200  # Límite máximo para la dirección (steer).
MAX_SPEED = 200  # Límite máximo para la velocidad (speed).

steerjoy = 0
speedjoy = 0
last_send_time = 0

# Inicialización de la conexión serial
try:
    ser = serial.Serial('/dev/serial0', 115200, timeout=1)  # Configuración del puerto serial de la Raspberry Pi a 115200 baudios con un timeout de 1 segundo.
except serial.SerialException as e:
    # Si ocurre un error al abrir el puerto, se muestra el mensaje de error y se detiene el programa.
    print(f"Error al abrir el puerto serial: {e}")
    exit(1)

app = Flask(__name__)
socketio = SocketIO(app, async_mode="gevent")     #3 Modos: gevent, eventlet, threading

# Iniciar la cámara de la Raspberry Pi con Picamera2
picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"size": (640, 480)})  # 640, 480 ; 1280, 720 ; 1920, 1080
picam2.configure(config)
picam2.start()
time.sleep(1)  # Permitir que la cámara se caliente


"""
# Función para capturar el video de la cámara
def gen_frames():
    while True:
        start_time = time.time()  # Comienza a medir el tiempo
        frame = picam2.capture_array()
        
        # Convertir de RGB a BGR para corregir los tonos azulados
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        
        ret, buffer = cv2.imencode('.jpg', frame)
        frame = buffer.tobytes()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
               
        # Controlar la tasa de cuadros
        elapsed_time = time.time() - start_time
        if elapsed_time < 0.04:  # Si ha pasado menos de 40ms, espera el resto del tiempo, 25fps
            time.sleep(0.04 - elapsed_time)
""" 
            
            
# Función para calcular el checksum
def calculate_checksum(start, steer, speed):
    # El checksum se calcula como la operación XOR entre los valores de start, steer, y speed,
    # y se limita a 16 bits sin signo (0xFFFF) para garantizar que no exceda este rango.
    return (start ^ steer ^ speed) & 0xFFFF

# Función para enviar los comandos a través de UART
def send_command(steer, speed):
    steer = int(max(-MAX_STEER, min(steer, MAX_STEER)))  # Asegurarse de que steer esté entre -MAX_STEER y MAX_STEER.
    speed = int(max(-MAX_SPEED, min(speed, MAX_SPEED)))  # Asegurarse de que speed esté entre -MAX_SPEED y MAX_SPEED.
    
    # Imprimir los valores actuales de steer y speed
    print(f"Enviando comandos - Steer: {steer}, Speed: {speed}")

    # Calcular el checksum para detectar posibles errores en la transmisión de datos.
    checksum = calculate_checksum(START_FRAME, steer, speed)

    # Crear el paquete de datos. El formato '<HhhH' especifica:
    # - H: unsigned short (2 bytes) para el start frame.
    # - h: signed short (2 bytes) para steer (dirección).
    # - h: signed short (2 bytes) para speed (velocidad).
    # - H: unsigned short (2 bytes) para el checksum.
    command = struct.pack('<HhhH', START_FRAME, steer, speed, checksum)

    # Enviar el paquete de datos a través del puerto UART.
    try:
        ser.write(command)
    except serial.SerialException as e:
        # Si ocurre un error durante la comunicación serial, se muestra un mensaje de error.
        print(f"Error de comunicación serial: {e}")
                  
@app.route('/')
def index():
    return render_template('index_joystick.html')

@app.route('/video_feed')
def video_feed():
    return Response(main_loop(), mimetype='multipart/x-mixed-replace; boundary=frame')

@socketio.on('control')
def handle_control(data):
    global steerjoy, speedjoy

    steerjoy = data.get('steer', 0)
    speedjoy = data.get('speed', 0)
    sliderx = data.get('SliderX', 0)
    slidery = data.get('SliderY', 0)
    
    # Asegurarse de que los valores estén dentro del rango adecuado
    steerjoy = int(max(-MAX_STEER, min(MAX_STEER, steerjoy)))
    speedjoy = int(max(-MAX_SPEED, min(MAX_SPEED, speedjoy)))
    
    #print(f"Steer: {steerjoy}, Speed: {speedjoy}, SliderX: {sliderx}, SliderY: {slidery}")
    
"""
# Función en segundo plano para enviar comandos continuamente
def continuous_command_loop():
    global steerjoy, speedjoy, last_send_time
    while True:
        # Enviar comandos continuamente
        current_time = time.time()
        # Si ha pasado el tiempo definido en TIME_SEND (50 ms), se envía un nuevo comando.
        if current_time - last_send_time >= TIME_SEND:
            send_command(steerjoy, speedjoy)  # Enviar los valores actuales de dirección y velocidad.
            last_send_time = current_time  # Actualizar el tiempo del último envío.
            
        # Un retardo de 10 ms es suficiente para evitar ciclos innecesarios y mantener el programa eficiente.
        time.sleep(0.01)
 """       
        
# Función en segundo plano para enviar comandos continuamente
def main_loop():
    global steerjoy, speedjoy, last_send_time
    while True:
        # Captura de video
        frame = picam2.capture_array()
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        ret, buffer = cv2.imencode('.jpg', frame)
        frame = buffer.tobytes()

        # Enviar fotograma de video
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

        # Enviar comandos a intervalos regulares
        current_time = time.time()
        # Si ha pasado el tiempo definido en TIME_SEND (50 ms), se envía un nuevo comando.
        if current_time - last_send_time >= TIME_SEND:
            send_command(steerjoy, speedjoy) # Enviar los valores actuales de dirección y velocidad.
            last_send_time = current_time   # Actualizar el tiempo del último envío.

        # Un retardo de 10 ms es suficiente para evitar ciclos innecesarios y mantener el programa eficiente.  
        time.sleep(0.01)


    
@socketio.on('disconnect')
def handle_disconnect():
    print("Cliente desconectado.")
    global steerjoy, speedjoy
    steerjoy = 0
    speedjoy = 0
    
@socketio.on('connect')
def handle_connect():
    print("Cliente conectado.")
    global steerjoy, speedjoy
    steerjoy = 0
    speedjoy = 0

if __name__ == '__main__':
    try:
        # Inicia el bucle de comandos en segundo plano
        socketio.start_background_task(main_loop)  
        # Corre la aplicación Flask
        socketio.run(app, host='192.168.1.35', port=4567)
    except KeyboardInterrupt:
        print("\nPrograma detenido por el usuario.")
        
    finally:
        picam2.stop()  # Detener la cámara de manera segura.
        if ser.is_open:
            ser.close()
            print("Puerto serial cerrado correctamente.")
