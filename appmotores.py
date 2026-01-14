from gevent import monkey    #Para gevent
monkey.patch_all()              
from flask import Flask, render_template, Response
from flask_socketio import SocketIO
from picamera2 import Picamera2
import cv2
import time
import serial
import struct
import RPi.GPIO as GPIO
import os

# Configuración del modo y el pin GPIO
GPIO.setmode(GPIO.BCM)
PIN = 18  # Con resistencia de 1kohm conectada al cable negro del boton de encendido.
duracion = 0.5
GPIO.setup(PIN, GPIO.OUT)

# Asegura que el pin comience en LOW
GPIO.output(PIN, GPIO.LOW)

START_FRAME = 0xABCD  # Definición del valor inicial (start frame) que indica el comienzo de un paquete de datos para garantizar una comunicación serial confiable.
TIME_SEND = 0.05    # Intervalo de tiempo (en segundos) entre el envío de comandos (50 ms).
MAX_STEER = 200  # Límite máximo para la dirección (steer).
MAX_SPEED = 200  # Límite máximo para la velocidad (speed).

steerjoy = 0
speedjoy = 0
batVoltage = 0
voltajereal = 0
speedR_meas = 0
speedL_meas = 0
cmd1 = 0
cmd2 = 0
last_send_time = 0

# Función para encender el GPIO por un instante
def pulso_gpio(pin, duracion):
    GPIO.output(pin, GPIO.HIGH)
    time.sleep(duracion)
    GPIO.output(pin, GPIO.LOW)
    
def speak(text):
    # Valores fijos de velocidad y tono
    speed = 1.2  # Velocidad normal es 1.0; mayor es más rápido    0.5 a 2
    pitch = 100  # Tono normal es 0; valores positivos/agudos, negativos/graves      -1000 a 1000
    # Generar archivo de audio, ajustar velocidad y tono, reproducir y eliminar archivo 
    os.system(f'pico2wave -l es-ES -w output.wav "{text}" && sox output.wav adjusted_output.wav tempo {speed} pitch {pitch} && aplay adjusted_output.wav && rm output.wav adjusted_output.wav')

# Inicialización de la conexión serial
try:
	# Pulso inicial al iniciar el programa
    print("Encendiendo GPIO al inicio...")
    pulso_gpio(PIN, duracion)
    
    ser = serial.Serial('/dev/ttyAMA0', 115200, timeout=2)  # Configuración del puerto serial de la Raspberry Pi a 115200 baudios con un timeout de 1 segundo.
    ser.reset_input_buffer()

except serial.SerialException as e:
    # Si ocurre un error al abrir el puerto, se muestra el mensaje de error y se detiene el programa.
    print(f"Error al abrir el puerto serial: {e}")
    exit(1)

app = Flask(__name__)
socketio = SocketIO(app, async_mode="gevent")     #3 Modos: gevent, eventlet, threading

# Iniciar la cámara de la Raspberry Pi con Picamera2
picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"size": (640, 360)})  # 640, 480 ; 1280, 720 ; 1920, 1080
picam2.configure(config)
picam2.start()
picam2.set_controls({"AfMode": 2})
time.sleep(1)  # Permitir que la cámara se caliente

speak("Seleccione la estación a donde debo llevar la carga.")

# Función para capturar el video de la cámara
def gen_frames():
	
    while True:
		
        start_time = time.time()  # Comienza a medir el tiempo
        
        frame = picam2.capture_array()
        
        # Convertir de RGB a BGR para corregir los tonos azulados
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        # Girar 90 grados en sentido horario
        # frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
        
        ret, buffer = cv2.imencode('.jpg', frame)
        frame = buffer.tobytes()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
               
        # Controlar la tasa de cuadros
        elapsed_time = time.time() - start_time
        if elapsed_time < 0.04:  # Si ha pasado menos de 40ms, espera el resto del tiempo, 25fps
            time.sleep(0.04 - elapsed_time)
            
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
    # print(f"Enviando comandos - Steer: {steer}, Speed: {speed}")

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
        

# Estructura para los datos de feedback
def parse_feedback(data):
    global batVoltage, speedR_meas, speedL_meas, cmd1, cmd2
	# El formato sigue el patrón: <HhhhhhhHH
    feedback = struct.unpack('<HhhhhhhHH', data)
    start, cmd1, cmd2, speedR_meas, speedL_meas, batVoltage, boardTemp, cmdLed, checksum = feedback
    calc_checksum = (start ^ cmd1 ^ cmd2 ^ speedR_meas ^ speedL_meas ^ batVoltage ^ boardTemp ^ cmdLed) & 0xFFFF

    # Verificar validez de los datos
    if start == START_FRAME and calc_checksum == checksum:
        #print(f"Received: start={start}, cmd1={cmd1}, cmd2={cmd2}, speedR_meas={speedR_meas}, "
        #      f"speedL_meas={speedL_meas}, batVoltage={batVoltage}, boardTemp={boardTemp}, cmdLed={cmdLed}, "
        #      f"received_checksum={checksum}, calculated_checksum={calc_checksum}")
        print(f"Steer = {cmd1}, Speed = {cmd2}, RPMD = {speedR_meas}, RPMI = {speedL_meas}, VB = {batVoltage/100:.1f}, km/h = {((speedL_meas-speedR_meas)/2)*0.0311:.1f}, TC = {boardTemp/10}\n")

# Función para recibir datos
def receive_feedback():
	if ser.is_open and ser.in_waiting >= 18:  # 18 bytes para toda la estructura
		data = ser.read(18)
		parse_feedback(data)

                  
@app.route('/')
def index():
    return render_template('index.html')
    
@app.route('/index.html')
def home():
    return render_template('index.html')
    
@app.route('/hmi.html')
def hmi():
    return render_template('hmi.html')
    
@app.route('/mode_manual.html/')
def manual():
    return render_template('mode_manual.html')
    
@app.route('/mode_person.html')
def mode_person():
    return render_template('mode_person.html')

@app.route('/mode_line.html')
def mode_line():
    return render_template('mode_line.html')

# Rutas para el control del administrador
@app.route('/calibrar_sensores.html')
def calibrar_sensores():
    return render_template('calibrar_sensores.html')

@app.route('/configurar_sensores.html')
def configurar_sensores():
    return render_template('configurar_sensores.html')

@app.route('/control_aceleracion.html')
def control_aceleracion():
    return render_template('control_aceleracion.html')
    
@app.route('/video_feed')
def video_feed():
    return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@socketio.on('control')
def handle_control(data):
    global steerjoy, speedjoy

    steerjoy = data.get('steer', 0)
    speedjoy = data.get('speed', 0)
    sliderx = data.get('SliderX', 0)
    slidery = data.get('SliderY', 0)
    
    # Asegurarse de que los valores estén dentro del rango adecuado
    # steerjoy = int(max(-MAX_STEER, min(MAX_STEER, steerjoy)))
    # speedjoy = int(max(-MAX_SPEED, min(MAX_SPEED, speedjoy)))
    
    #print(f"Steer: {steerjoy}, Speed: {speedjoy}, SliderX: {sliderx}, SliderY: {slidery}")
    
    
# Función en segundo plano para enviar comandos continuamente
def continuous_command_loop():
    global steerjoy, speedjoy, last_send_time, batVoltage, voltajereal
    while True:
		# Recibir datos
        receive_feedback()
        
        if ser.is_open:
            ser.reset_input_buffer()

        # Enviar comandos continuamente
        current_time = time.time()
        # Si ha pasado el tiempo definido en TIME_SEND (50 ms), se envía un nuevo comando.
        if current_time - last_send_time >= TIME_SEND:
            send_command(steerjoy, speedjoy)  # Enviar los valores actuales de dirección y velocidad.
            last_send_time = current_time  # Actualizar el tiempo del último envío.
            
        voltajereal = batVoltage/100
        
        socketio.emit('battery_update', {'batVoltage': voltajereal}) 
        # Un retardo de 10 ms es suficiente para evitar ciclos innecesarios y mantener el programa eficiente.
        time.sleep(0.01)

    
@socketio.on('disconnect')
def handle_disconnect():
    print("Control manual desactivado.")
    #speak("Control manual desactivado.")
    global steerjoy, speedjoy
    steerjoy = 0
    speedjoy = 0
    
@socketio.on('connect')
def handle_connect():
    print("Control manual activado.")
    #speak("Control manual activado.")
    global steerjoy, speedjoy
    steerjoy = 0
    speedjoy = 0

if __name__ == '__main__':
    try:
        # Inicia el bucle de comandos en segundo plano
        socketio.start_background_task(continuous_command_loop)  
        # Corre la aplicación Flask
        socketio.run(app, host='192.168.58.4', port=4567) #Importante verificar la IP de la red a la que esta conectada la RPi y el celular
    except KeyboardInterrupt:                             #sudo ifconfig wlan0 down
        speak("El proceso fue interrumpido.")
        print("\nPrograma detenido por el usuario.")
        
    finally:
        picam2.stop()  # Detener la cámara de manera segura.
        
        # Pulso final antes de salir
        pulso_gpio(PIN, duracion)
        # Limpieza de los GPIO
        GPIO.cleanup()
        
        if ser.is_open:
            ser.close()
            print("Puerto serial cerrado correctamente.")



## sudo apt install python3-
