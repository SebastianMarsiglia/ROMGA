from gevent import monkey
monkey.patch_all()
import numpy as np
import struct
import serial
import json
import time
import cv2
from flask import Flask, render_template, request, redirect, session, Response,jsonify
from flask_socketio import SocketIO, disconnect
from picamera2 import Picamera2 #En ubuntu 22.04 no pude hacer que funcione picamera2. Requiere camara usb estandar wide autofocus.
import os
from functools import wraps
import pigpio
import subprocess
import uuid
import mediapipe as mp
import threading


person_mode_active = False
person_mode_paused = False
person_task = None  # al principio del script
if subprocess.run(["pgrep", "pigpiod"]).returncode != 0: subprocess.run(["sudo", "pigpiod"]); time.sleep(2)
pi = pigpio.pi()
# Verifica si pigpio está corriendo
if not pi.connected:
    print("No se pudo conectar al demonio pigpio.")
    exit()
active_user = {
    'username': None,
    'token': None
}
# ==== GPIO ====
PIN = 18

pi.set_mode(PIN,pigpio.OUTPUT)
pi.write(PIN,0)
DURACION = 0.5

def pulso_gpio(pin, dur):
    pi.write(PIN,1)
    time.sleep(dur)
    pi.write(PIN,0)
pin = 7 #IN1 - Rele 1 - Luz delantera
pin2 = 8 #IN2 - Rele 2 - Luz trasera
pi.set_mode(pin, pigpio.OUTPUT)
pi.set_mode(pin2, pigpio.OUTPUT)
pi.write(pin, 1) 
pi.write(pin2, 1)
# ==== UART ====
SERIAL_PORT = '/dev/ttyAMA0'
BAUDRATE = 115200
FRAME_SIZE = 18
START_FRAME = 0xABCD
TIME_SEND = 0.05
MAX_STEER = 250
MAX_SPEED = 250
steer_output = 0
speed_output = 0
steerjoy = 0
speedjoy = 0
manual_mode = False
manual_task = None
battery_task_started = False
last_send_time = 0

try:
    pulso_gpio(PIN, DURACION)
    ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=2)
    ser.reset_input_buffer()
except serial.SerialException as e:
    print(f"Error abriendo UART: {e}")
    exit(1)
"""
#=========ESP32=======
try:
    # Inicializar conexión serial
    ESP32 = serial.Serial('/dev/ttyACM0', 115200, timeout=2)
    time.sleep(1)
    print("✅ ESP32 conectado")
except serial.SerialException:
    ESP32=NotImplemented
    print("❌ Error al conectar ESP32")
    exit()

# Límites para alarmas
PESO_MAX = 150000        # gramos
ANGULO_MAX = 20.0        # grados
DISTANCIA_MIN = 50.0     # cm
TEMP_MAX = 50.0          # °C
TEMP_MIN = 10.0
p1 = 0 
d1 = 0 
d2 = 0 
t1 = 0 
aX = 0 
aY = 0 
aZ = 0

def actualizarsensado():
    global PESO_MAX, ANGULO_MAX, DISTANCIA_MIN, TEMP_MAX, TEMP_MIN, p1, d1, d2, t1, aX, aY, aZ,ESP32
    while True:
        if ESP32 and ESP32.in_waiting > 0:
            # Leer línea enviada por ESP32
            line = ESP32.readline().decode('utf-8').rstrip()
            time.sleep(0.1)
            if line:
                try:
                    data = json.loads(line)
                except json.JSONDecodeError as e:
                    print(f"[ERROR JSON] Linea malformada: {line} - {e}")
                    return              

                # Asignar valores
                p1 = data["p1"]
                d1 = data["d1"]
                d2 = data["d2"]
                t1 = data["t1"]
                aX = data["aX"]
                aY = data["aY"]
                aZ = data["aZ"]

                # Imprimir datos
                print("Peso:", p1 / 1000, "kg")
                print("Distancia 1:", d1, "cm")
                print("Distancia 2:", d2, "cm")
                print("Temp Bat:", t1, "°C")
                print("Ángulo X:", aX, "°")
                print("Ángulo Y:", aY, "°")

                #if p1 >= PESO_MAX:
                #print("🚨 ¡Peso máximo excedido!")
                if t1 >= TEMP_MAX or t1 <= TEMP_MIN:
                    print("🌡️ ¡Temperatura de batería fuera de rango!")
                if abs(aY) > ANGULO_MAX:
                    print("⚠️ ¡Inclinación frontal peligrosa!")
                if abs(aX) > ANGULO_MAX:
                    print("⚠️ ¡Inclinación lateral peligrosa!")
                if d1 < DISTANCIA_MIN or d2 < DISTANCIA_MIN:
                    #    steer = 0
                    #    speed = 0
                    print("🚧 ¡Obstáculo detectado cerca!")

                ESP32.write(b"Recibido\n")
"""                   

# ==== App y Cámara ====
app = Flask(__name__)
app.secret_key = 'clave_romga_secreta'
socketio = SocketIO(app, async_mode='gevent')

picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"size": (640, 360)})
picam2.configure(config)
picam2.start()
picam2.set_controls({"AfMode": 2})

time.sleep(1)

# ==== Voz ====
def speak(text):
    os.system(f'pico2wave -l es-ES -w output.wav "{text}" && sox output.wav adjusted_output.wav tempo 1.2 pitch 100 && aplay adjusted_output.wav && rm output.wav adjusted_output.wav')

# ==== UART Parsing ====
def send_command(steer, speed):
    chk = (START_FRAME ^ steer ^ speed) & 0xFFFF
    pkt = struct.pack('<HhhH', START_FRAME, steer, speed, chk)
    try:
        ser.write(pkt)
    except serial.SerialException as e:
        print(f"[ERROR] Serial write: {e}")


batVoltage = 0
kphora = 0

def parse_feedback(data):
    global batVoltage, kphora
    fb = struct.unpack('<HhhhhhhHH', data)
    start, cmd1, cmd2, spR, spL, rawB, tmp, led, chk = fb
    calc = (start ^ cmd1 ^ cmd2 ^ spR ^ spL ^ rawB ^ tmp ^ led) & 0xFFFF
    if start == START_FRAME and calc == chk:
        batVoltage = round(rawB/100, 1)
        kphora = ((spL-spR)/2)*0.0311

        print(f"Steer = {cmd1}, Speed = {cmd2}, RPMD = {spR}, RPMI = {spL}, VB = {batVoltage:.1f}, km/h = {kphora:.1f}\n")


def receive_feedback():
    try:
        if ser.is_open and ser.in_waiting >= FRAME_SIZE:
            data = ser.read(FRAME_SIZE)
            if len(data) == FRAME_SIZE:
                parse_feedback(data)
    except serial.SerialException as e:
        print(f"[WARN] Serial read failed: {e}")

# ==== Background Loops ====
def battery_loop():
    global batVoltage #, kphora, p1, d1, d2, t1, aX, aY, aZ
    while True:
        receive_feedback()
        if ser.is_open:
            ser.reset_input_buffer()        
        time.sleep(0.1)
        #actualizarsensado()
        #time.sleep(0.1)
        socketio.emit('battery_update', {'batVoltage': batVoltage})
        #socketio.emit('km/h_update', {'km/h': kphora}) #Falta integrar en HMI

        #socketio.emit('d1_update', {'Distancia1': d1}) #Falta integrar en HMI
        #socketio.emit('d2_update', {'Distancia2': d2}) #Falta integrar en HMI
        
        
######Servo

# Define el pin GPIO 12 donde se conecta el servomotor
gpio_pin = 12

# Configura el GPIO 12 como salida PWM
pi.set_mode(gpio_pin, pigpio.OUTPUT)

# Función para mover el servomotor a un ángulo
def servo(angulo):
    angulo=int(max(1, min(130, angulo)))
    # Convierte el ángulo (0-180) a un pulso en microsegundos (500-2500)
    pulso = ((angulo / 180) * (2500 - 500) + 500)
    pi.set_servo_pulsewidth(gpio_pin, pulso)

    #dutycycle = int((pulso / 20000) * 1000000)
    #pi.hardware_PWM(gpio_pin, 50, dutycycle) #0-1000000 0-100% 50hz 20ms 0.5ms-2.5ms 500us-2500us 2.5%-12.5% 25000-125000

# Función para mover el servomotor suavemente de un ángulo a otro
def servo2(angulo_inicial, angulo_final, pasos=100, tiempo_entre_pasos=0.003):
    """ Mueve el servomotor de angulo_inicial a angulo_final suavemente en pasos """
    paso = (angulo_final - angulo_inicial) / pasos  # Calcula el tamaño de cada paso
    
    for i in range(pasos):
        angulo_actual = angulo_inicial + paso * i
        servo(angulo_actual)  # Mueve el servomotor al ángulo calculado
        time.sleep(tiempo_entre_pasos)  # Espera entre cada paso
        
#####Fin Servo


#loop manual
def control_loop():
    global last_send_time
    last_send_time = 0
    servo(110)
    while manual_mode:
        now = time.time()
        if now - last_send_time >= TIME_SEND:
            steer = int(max(-MAX_STEER, min(MAX_STEER, steerjoy)))
            speed = int(max(-MAX_SPEED, min(MAX_SPEED, speedjoy)))
            send_command(steer, speed)
            last_send_time = now
        time.sleep(0.01)
#loop linea
def control_loop_linea():
    global last_send_time2, line_mode_active, line_pause, steer_output, speed_output  # <-- Faltaban en tu c�digo
    last_send_time2 = 0
    while True:  # Siempre activo
        if not line_mode_active:
            time.sleep(0.1)
            continue

        if line_pause:
            send_command(0, 0)
            time.sleep(0.1)
            continue

        now2 = time.time()
        if now2 - last_send_time2 >= TIME_SEND:
            steer = int(max(-MAX_STEER, min(MAX_STEER, steer_output)))
            speed = int(max(-MAX_SPEED, min(MAX_SPEED, speed_output)))
            send_command(steer, speed)
            last_send_time2 = now2
        time.sleep(0.01)


# ==== Procesamiento de Imagen ====
def preprocess_image(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    _, binary = cv2.threshold(blurred, 50, 255, cv2.THRESH_BINARY_INV) #s
    kernel = np.ones((5, 5), np.uint8)
    eroded = cv2.erode(binary, kernel, iterations=1)
    dilated = cv2.dilate(eroded, kernel, iterations=1)
    return dilated

def detect_line(binary_frame):
    frame_height, frame_width = binary_frame.shape
    roi = binary_frame[int(frame_height / 2):, :]
    contours, _ = cv2.findContours(roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    return contours, roi, frame_width, frame_height

MAX_STEERline = 40

def compute_steer(cx, frame_center):
    return ((cx - frame_center) / frame_center) * MAX_STEERline

def detect_arucos(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_50)
    parameters = cv2.aruco.DetectorParameters()
    parameters.adaptiveThreshWinSizeMin = 3
    parameters.adaptiveThreshWinSizeMax = 23
    parameters.adaptiveThreshWinSizeStep = 5
    parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
    corners, ids, _ = detector.detectMarkers(gray)
    if ids is not None:
        frame = cv2.aruco.drawDetectedMarkers(frame, corners, ids)
    return frame, ids

steerPID = 0.0
prev_steer = 0.0
dt_pid=0.1
kp_pid = 0.6
kd_pid = 0.2
def pid_control(steer):
    global steerPID, prev_steer, kp_pid, kd_pid, dt_pid
    steerP = steer
    steerD = (steer - prev_steer) / dt_pid
    steerPID = (steerP * kp_pid) + (steerD * kd_pid)
    steerPID = int(max(-MAX_STEERline, min(MAX_STEERline, steerPID)))
    prev_steer = steer
    return steerPID


# ==== Login System ====
def login_required(f):
    @wraps(f)
    def wrap(*args, **kwargs):
        if 'usuario' not in session:
            return redirect('/')
        return f(*args, **kwargs)
    return wrap

@app.route('/')
def login_page():
    return render_template('index.html')

@app.route('/login', methods=['POST'])
def login():
    global active_user
    u = request.form['usuario']
    p = request.form['contrasena']

    if (u, p) in [('Operador', '1234'), ('Administrador', 'admin')]:
        # SI HAY UN USUARIO ACTIVO, NO DEJA INGRESAR
        if active_user['username'] is not None:
            return render_template('index.html', error='Ya hay un usuario activo en el sistema.')
        token = str(uuid.uuid4())
        session['usuario'] = u
        session['token'] = token
        active_user['username'] = u
        active_user['token'] = token
        print(f"[INFO] Usuario {u} ha iniciado sesion con token {token}")
        return redirect('/hmi')

    return render_template('index.html', error='Usuario o contraseña incorrecta')



@app.route('/logout')
def logout():
    global active_user
    if ('usuario' in session and
        'token' in session and
        active_user['username'] == session['usuario'] and
        active_user['token'] == session['token']):
        active_user['username'] = None
        active_user['token'] = None
    session.clear()
    return redirect('/')



# ==== Rutas ====
@app.route('/hmi')
@login_required
def hmi():
    picam2.set_controls({"AfMode": 2})
    global manual_mode
    manual_mode = False
    return render_template('hmi.html')

@app.route('/mode_manual')
@login_required
def mode_manual():
    picam2.set_controls({
    #"AeEnable": True,
    "ExposureTime": 0,      # 0 permite que el sistema decida
    "AnalogueGain": 0       # 0 tambien devuelve el control automatico del ISO
})
    global manual_mode, manual_task,line_mode_active, person_mode_active, person_task
    line_mode_active = False
    person_mode_active = False
    person_task = None
    speak("Modo manual activado")
    if not manual_mode:
        manual_mode = True
        manual_task = socketio.start_background_task(control_loop)
    return render_template('mode_manual.html')

@app.route('/mode_line')
@login_required
def mode_line():
    picam2.set_controls({
    #"AeEnable": True,        # Desactiva el auto-exposure
    "AnalogueGain": 200      # Aproximadamente ISO 200, DEPENDE DEL COLOR DEL SUELO.
})
    global manual_mode, person_mode_active, person_task
    manual_mode = False
    person_mode_active = False
    person_task = None
    
    
    speak("Modo seguidor de línea activado")
    return render_template('mode_line.html')
@app.route('/mode_person')
@login_required
def mode_person():
    picam2.set_controls({
    #"AeEnable": True,
    "ExposureTime": 0,      # 0 permite que el sistema decida
    "AnalogueGain": 0       # 0 tambien devuelve el control automatico del ISO
})
    global manual_mode, line_mode_active, person_mode_active, person_mode_paused, person_task

    # Desactivar otros modos
    manual_mode = False
    line_mode_active = False

    # Activar seguimiento de persona
    person_mode_active = True
    person_mode_paused = False

    # Mensaje de voz
    speak("Modo seguimiento de persona activado")

    # Solo iniciar el hilo si a�n no se ha iniciado o termin�
    if person_task is None or not person_task.is_alive():
        person_task = socketio.start_background_task(seguimiento_persona_loop)

    return render_template('mode_person.html')

@app.route('/video_feed')
@login_required
def video_feed():
    def gen():
        while True:
            frame = picam2.capture_array()
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            ret, buf = cv2.imencode('.jpg', frame)
            yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + buf.tobytes() + b'\r\n')
            time.sleep(0.04)
    return Response(gen(), mimetype='multipart/x-mixed-replace; boundary=frame')
#====Botones importantes
@app.route('/reiniciar_controladora', methods=['POST'])
def reiniciar_controladora():
    try:
        pulso_gpio(PIN, DURACION)
        return jsonify({'success': True, 'message': 'Controladora reiniciada'})
    except Exception as e:
        return jsonify({'success': False, 'message': str(e)}), 500# Ruta para apagar la Raspberry Pi de forma segura


# ==== WebSocket Seguimiento línea ====
target_station = None
line_mode_active = False
line_pause = False

@socketio.on('line_follow_station')
def start_line_tracking(data):
    global target_station, line_mode_active, line_pause, manual_mode
    target_station = data.get('estacion')
    manual_mode = False
    if target_station is not None:
        line_mode_active = True
        line_pause = False
        speak(f"Estacion {target_station} seleccionada")
        socketio.start_background_task(seguimiento_loop)


@socketio.on('line_follow_pause')
def pause_line():
    global line_pause
    line_pause = True
    send_command(0,0)
@socketio.on('line_follow_resume')
def resume_line():
    global line_pause
    line_pause = False
    
@socketio.on('line_follow_stop')
def stop_line():
    pi.write(pin2, 1) #apagar
    pi.set_servo_pulsewidth(gpio_pin, 0)
    global line_mode_active
    line_mode_active = False



def seguimiento_loop():
    global target_station, line_mode_active, line_pause, steer_output, speed_output, last_send_time #, d1, d2
    pi.write(pin2, 0) #Encender luz delanter
    steer_output = 0
    speed_output = 0
    marker_id = 0
    last_send_time = 0
    acel = 2
    areamin = 3000
    areamax = 30000
    timesteerhold = 1
    last_send_timeLine = 0
    last_cx = None
    # Define el pin GPIO 12 donde se conecta el servomotor
    servo(1)    # Mueve suavemente de 1 grados
    while line_mode_active:
        if line_pause:
            time.sleep(0.1)
            continue

        frame = picam2.capture_array()
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        binary_frame = preprocess_image(frame)
        contours, roi, frame_width, frame_height = detect_line(binary_frame)
        roi_offset =int(frame_height/2)
        frame_center = frame_width // 2

        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest_contour)
            if areamax > M['m00'] > areamin:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00']) +roi_offset
                steer = compute_steer(cx, frame_center)
                # Dibuja la línea y el círculo en el centroide detectado
                cv2.line(frame, (cx, cy), (frame_center, cy), (0, 255, 0), 6)
                cv2.circle(frame, (cx, cy), 8, (0, 0, 255), -1)
                last_cx = cx
                steer_output = pid_control(steer)
                last_send_timeLine = time.time()
                speed_output += acel
            else:
                if speed_output > 0:
                    speed_output -= acel
                if last_cx is not None:
                    steer_output = steerPID
        else:
            if speed_output > 0:
                speed_output -= acel
            if last_cx is not None:
                steer_output = steerPID

        if time.time() - last_send_timeLine >= timesteerhold:
            steer_output = 0
        #speed_output = int(max(-MAX_SPEED, min(MAX_SPEED, speed_output)))
        speed_output = int(max(0, min(40, speed_output)))
        frame, ids = detect_arucos(frame)

        if ids is not None:
            for i in range(len(ids)):
                marker_id = ids[i][0]

        if marker_id == target_station:
            speak(f"Estación {target_station} alcanzada. Deteniendo el robot.")
            steer_output = 0
            speed_output = 0
            line_pause=True
        if time.time() - last_send_time >= TIME_SEND:
            send_command(steer_output, speed_output)
            last_send_time = time.time()

    
        cv2.drawContours(frame[roi_offset:, :], contours, -1, (0, 255, 0), 3) # Contornos de linea
        
        
        _, jpeg = cv2.imencode('.jpg', frame)
        frame_bytes = jpeg.tobytes()
        socketio.emit('video_frame_linea', frame_bytes)
        time.sleep(0.04)
##Agregar la busqueda de la linea sino es detectada por mas de 5s
# ==== SocketIO manual control ====
@socketio.on('manual_servo_angle')
def handle_servo_angle(data):
    angle = int(data.get('angle', 0))
    angle = max(0, min(130, angle))  # Seguridad por si acaso
    servo(angle)  # Llama a tu funcion ya definida para mover el servo
led_state = True  # True: Encendido, False: Apagado
@app.route('/toggle_led', methods=['POST'])
def toggle_led():
    global led_state
    try:
        led_state = not led_state
        pi.write(pin, int(led_state))  # pin ya definido como IN1 barra LED
        return jsonify({
            'success': True,
            'state': led_state,
            'message': 'LED Encendido' if led_state else 'LED Apagado'
        })
    except Exception as e:
        return jsonify({'success': False, 'message': str(e)}), 500

@socketio.on('connect')
def on_connect():
    global battery_task_started #,ESP32
    if not battery_task_started:
        socketio.start_background_task(battery_loop)
        battery_task_started = True
        



@socketio.on('control')
def on_control(data):
    global steerjoy, speedjoy
    steerjoy = data.get('steer', 0)
    speedjoy = data.get('speed', 0)

@socketio.on('disconnect')
def on_disconnect():
    global active_user, steerjoy, speedjoy
    steerjoy=0
    speedjoy=0
    
    print("[SOCKET] Usuario desconectado. Liberando sesion activa.")
    active_user['username'] = None
    active_user['token'] = None



#=================Seguimiento de persona===
# Inicializar MediaPipe Pose y utilidades de dibujo
mp_pose = mp.solutions.pose
mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles

def seguimiento_persona_loop():
    global steer, speed, person_mode_active, person_mode_paused
    velocidad_deseada = 100 #rpm 1.8km/h
    max_steer = 100
    

        # Variables globales para compartir el frame capturado entre hilos
    frame = None             # Frame compartido
    lock = threading.Lock()  # Bloqueo para evitar acceso simultáneo
    new_frame = False        # Bandera que indica nuevo frame disponible


# Variables globales para PID
    steer = 0.0
    prev_steer = 0.0  # Variable para almacenar el valor anterior de 'steeri"

    # Función para implementar el control PIDs
    def pid_control(steeri, kp=0.6, kd=0.1, dt=0.1):   #0.6 y 0.08
        global steer, prev_steer
        
        # Componente proporcional
        steerP = steeri
        # Componente derivativa (derivada aproximada)
        steerD = (steeri - prev_steer) / dt

        # PID final
        steer = (steerP * kp) + (steerD * kd)

        # Limitar Steer entre -500 y 500
        steer = int(max(-max_steer, min(max_steer, steer)))

        # Actualizar el valor previo de 'steer'
        prev_steer = steeri

        return steer


    # Lista de índices de puntos clave del torso (hombro izquierdo y cadera izquierda)
    torso_keypoints = [11, 12, 15, 16, 23, 24]

    # Variables de control
    steeri = 0
    speed = 0
    hombro_iy = 0
    cadera_iy = 360
    distancia = 0
    distancia_min = 1.9
    distancia_max = 3.5

    acel = 6
    flagmanos = -1
    manos_arriba_anterior = False
    servo(128)
 
    while person_mode_active:
       
      #  global steer, steeri, speed, acel, hombro_iy, cadera_iy, distancia, distancia_min, distancia_max, velocidad_deseada, max_steer, new_frame, frame, flagmanos, manos_arriba_anterior

        # Configuración de MediaPipe Pose (configuración ultra-ligera)
        with mp_pose.Pose(
            model_complexity=0, #Probar con 0 y 1.       0 es optimizado, 1 confiable.
            min_detection_confidence=0.8,
            min_tracking_confidence=0.8
        ) as pose:
            prev_time = time.time()  # Para el cálculo de FPS
            
            while person_mode_active:
                bgr_frame = picam2.capture_array()
                height, width, _ = bgr_frame.shape
                frame_center = width // 2
                rgb_frame = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2RGB)
                results = pose.process(rgb_frame)

                # Procesar la imagen con MediaPipe
                # results = pose.process(bgr_frame)
                
                cadera_iy = 360
                hombro_iy = 0

                if results.pose_landmarks:
                
                    # Obtener landmarks relevantes
                    hombro_izq = results.pose_landmarks.landmark[11]
                    muñeca_izq = results.pose_landmarks.landmark[15]

                    hombro_der = results.pose_landmarks.landmark[12]
                    muñeca_der = results.pose_landmarks.landmark[16]

                    # Verificar si ambas manos están arriba con un margen del 10%
                    margen = 0.1  # 10% de altura de imagen, # margen del 10% del alto de la imagen normalizada (0 a 1) - o + depende de orientacion, probar
                    manos_arriba = (muñeca_izq.y < (hombro_izq.y - margen)) and (muñeca_der.y < (hombro_der.y - margen))

                    if manos_arriba != manos_arriba_anterior and manos_arriba == True and muñeca_izq.visibility > 0.6 and muñeca_der.visibility > 0.6:
                        flagmanos = -flagmanos
                        print(" Ambas manos están arriba")
                        speak("Comando recibido, manos arriba")
                    manos_arriba_anterior = manos_arriba
                    
                    # Extraer las coordenadas de la cadera izquierda (índice 23) y el hombro izquierdo (índice 11)
                    cadera = results.pose_landmarks.landmark[23]
                    cadera_iy = cadera.y * height

                    hombro = results.pose_landmarks.landmark[11]
                    hombro_iy = hombro.y * height

                    # Evitar división por cero en caso poco probable de que ambos puntos estén al mismo nivel
                    delta_y = abs(cadera_iy - hombro_iy)
                    distancia = 200 / delta_y if delta_y != 0 else 0

                    # Calcular steer en función de la posición horizontal de la cadera izquierda
                    if flagmanos == 1: 
                        steeri = (((cadera.x*width) - frame_center) / frame_center) * max_steer
                        steer = pid_control(steeri)
                    else:
                        steer = 0
                        if speed > 0: 
                            speed -= acel*3

                    # Actualizar la velocidad en función de la distancia y velocidad deseada
                    if speed < velocidad_deseada and distancia > distancia_min and distancia < distancia_max and flagmanos == 1:
                        speed += acel
                    if distancia <= distancia_min or distancia >= distancia_max:
                        if speed > 0: 
                            speed -= acel*3

                    #if distancia < distancia_min:
                     #   socketio.emit('alerta', {'msg': 'Persona demasiado cerca'})
                    #elif distancia > distancia_max:
                      #  socketio.emit('alerta', {'msg': 'Persona demasiado lejos'})


                    # Dibujar puntos clave del torso
                    for i in torso_keypoints:
                        landmark = results.pose_landmarks.landmark[i]
                        x, y = int(landmark.x * width), int(landmark.y * height)
                        cv2.circle(rgb_frame, (x, y), 5, (0, 255, 0), -1)
                else:
                    distancia = 1.0
                    steer = 0
                    flagmanos = -1
                    if speed > 0: 
                        speed -= acel*3
                    #socketio.emit('alerta', {'msg': 'Persona no detectada'})

                    
                if speed < 0:
                    speed = 0
                # Mostrar valores en consola
                print(f"Steer = {steer:.1f}, Speed = {speed:.1f}, Distancia = {distancia:.1f} m, Flagmanos= {flagmanos}")
                send_command(steer, speed)

                # Calcular y mostrar FPS
                curr_time = time.time()
                fps = 1 / (curr_time - prev_time) if curr_time != prev_time else 0
                prev_time = curr_time
                cv2.putText(rgb_frame, f'FPS: {int(fps)}', (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
                    
                # Enviar video a cliente web
                _, jpeg = cv2.imencode('.jpg', rgb_frame)
                frame_bytes = jpeg.tobytes()
                socketio.emit('video_frame_persona', frame_bytes)

                time.sleep(0.04)

@socketio.on('person_follow_start')
def start_person_follow():
    global person_mode_active, person_mode_paused, manual_mode, line_mode_active, person_task
    manual_mode = False
    line_mode_active = False
    person_mode_active = True
    person_mode_paused = False
    print("Modo seguimiento de persona activado")
    if person_task is None or not person_task.is_alive():
        person_task = socketio.start_background_task(seguimiento_persona_loop)


@socketio.on('person_follow_pause')
def pause_person_follow():
    global steer, speed, person_mode_active, person_mode_paused, person_task
    steer=0
    speed=0
    person_mode_active = False
    person_mode_paused = True
    send_command(steer, speed)
    person_task = None

    speak("Modo seguimiento de persona pausado")

@socketio.on('person_follow_resume')
def resume_person_follow():
    global person_mode_paused, person_mode_active, person_task
    person_mode_paused = False
    person_mode_active = True
    speak("Modo seguimiento de persona reanudado")

    # Solo iniciar el hilo si a�n no se ha iniciado o termin�
    if person_task is None or not person_task.is_alive():
        person_task = socketio.start_background_task(seguimiento_persona_loop)
    

@socketio.on('person_follow_stop')
def stop_person_follow():
    global person_mode_active, person_mode_paused,person_task
    person_mode_active = False
    person_mode_paused = True
    send_command(0, 0)
    person_task = None
    speak("Modo seguimiento de persona detenido")
    pi.write(pin, 1) 
# PID params globales
kp_pid = 0.7
kd_pid = 0.2

@app.route('/update_pid', methods=['POST'])
def update_pid():
    global kp_pid, kd_pid
    data = request.get_json()
    param = data.get('param')
    value = data.get('value')

    if param == 'kp':
        kp_pid = value
    elif param == 'kd':
        kd_pid = value
    return jsonify({'success': True})
def cerrar_recursos():
    pass
#========ip dinamica
import socket
def get_ip_address():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        # Direccion ficticia, no se envia nada, solo para obtener IP local
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
    except Exception:
        ip = "127.0.0.1"
    finally:
        s.close()
    return ip
print(f"\n[INFO] Servidor iniciado en http://{get_ip_address()}:4567")
# ==== Main ====
if __name__ == '__main__':
    try:
        socketio.run(app, host=get_ip_address(), port=4567)
    finally:
        picam2.stop()
        pi.write(pin, 1) 
        pi.write(pin2, 1) 
        pulso_gpio(PIN, DURACION)
        # Detén el servomotor y apaga la conexión de pigpio
        pi.set_servo_pulsewidth(gpio_pin, 0)
        pi.stop()
        if ser.is_open:
            ser.close()
            ser.close()
