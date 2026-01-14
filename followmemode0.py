#Codigo para seguir una persona, debe haber buena iluminacion, y los contornos de su ropa se deben diferenciar del ambiente.
#Falta agregar el id de una persona cercana especifica, para que no cambie a otra persona durante el recorrido.

import cv2
import mediapipe as mp
from picamera2 import Picamera2
import time
import threading
import pigpio
import subprocess
import struct
import serial
import os

if subprocess.run(["pgrep", "pigpiod"]).returncode != 0: subprocess.run(["sudo", "pigpiod"]); time.sleep(2)
pi = pigpio.pi()
# Verifica si pigpio está corriendo
if not pi.connected:
    print("No se pudo conectar al demonio pigpio.")
    exit()

# ==== GPIO ====
PIN = 18
pi.set_mode(PIN,pigpio.OUTPUT)
pi.write(PIN,0)
DURACION = 0.5

def pulso_gpio(pin, dur):
    pi.write(PIN,1)
    time.sleep(dur)
    pi.write(PIN,0)
#@reboot sleep 15 && python3 /home/sebasmarsiglia2024/Descargas/LLAMDAS_copy/followmemode1.py


pin = 7 #IN1 - Rele 1 - Luz delantera
pin2 = 8 #IN2 - Rele 2 - Luz trasera
pi.set_mode(pin, pigpio.OUTPUT)
pi.set_mode(pin2, pigpio.OUTPUT)
pi.write(pin, 0) 
pi.write(pin2, 1)

        
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


# ==== UART ====
SERIAL_PORT = '/dev/ttyAMA0'
BAUDRATE = 115200
FRAME_SIZE = 18
START_FRAME = 0xABCD
TIME_SEND = 0.05
velocidad_deseada = 100 #rpm 1.8km/h
max_steer = 100


try:
    pulso_gpio(PIN, DURACION)
    time.sleep(1)
    ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=2)
    ser.reset_input_buffer()
except serial.SerialException as e:
    print(f"Error abriendo UART: {e}")
    exit(1)

# ==== UART Parsing ====
def send_command(steerc, speedc):
    steerc = int(steerc)  # Asegura que steer es entero
    speedc = int(speedc)  # Asegura que speed es entero
    chk = (START_FRAME ^ steerc ^ speedc) & 0xFFFF
    pkt = struct.pack('<HhhH', START_FRAME, steerc, speedc, chk)
    try:
        ser.write(pkt)
    except serial.SerialException as e:
        print(f"[ERROR] Serial write: {e}")


# Inicializar MediaPipe Pose y utilidades de dibujo
mp_pose = mp.solutions.pose
mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles

# Configurar la cámara a 360p
picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"size": (640, 360), "format": "RGB888"}) #640, 360 funciona bien, a menos resolucion pierde la persona mas seguido.
picam2.configure(config)
picam2.set_controls({"AfMode": 2})
picam2.start()

# Variables globales para compartir el frame capturado entre hilos
frame = None             # Frame compartido
lock = threading.Lock()  # Bloqueo para evitar acceso simultáneo
new_frame = False        # Bandera que indica nuevo frame disponible

# Variables globales para PID
steer = 0.0
prev_steer = 0.0  # Variable para almacenar el valor anterior de 'steeri"

# Función para implementar el control PID
def pid_control(steeri, kp=0.5, kd=0.07, dt=0.1):   #0.6 y 0.08
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
    
# ==== Voz ====
def speak(text):
    os.system(f'pico2wave -l es-ES -w output.wav "{text}" && sox output.wav adjusted_output.wav tempo 1.2 pitch 100 && aplay adjusted_output.wav && rm output.wav adjusted_output.wav')
    
    
def capture_thread():
    """Captura de imágenes en un hilo separado."""
    global frame, new_frame
    while True:
        temp_frame = picam2.capture_array()
        with lock:
            frame = temp_frame
            new_frame = True  # Se marca que hay un nuevo frame disponible
        time.sleep(0.1)  # dentro de capture_thread
        
def detectar_chaleco_naranja(rgb_frame, landmarks, h, w):
    # Evalua si hay chaleco naranja en el torso (entre ambos hombros y ambas caderas).
    try:
        # Coordenadas de hombros y caderas
        hombro_izq = landmarks[11]
        hombro_der = landmarks[12]
        cadera_izq = landmarks[23]
        cadera_der = landmarks[24]

        # Convertir a coordenadas de imagen
        x1 = int(min(hombro_izq.x, cadera_izq.x) * w)
        x2 = int(max(hombro_der.x, cadera_der.x) * w)
        y1 = int(min(hombro_izq.y, hombro_der.y) * h)
        y2 = int(max(cadera_izq.y, cadera_der.y) * h)

        # Limitar el ROI dentro de la imagen
        x1 = max(0, x1)
        x2 = min(w - 1, x2)
        y1 = max(0, y1)
        y2 = min(h - 1, y2)

        torso_roi = rgb_frame[y1:y2, x1:x2]

        hsv = cv2.cvtColor(torso_roi, cv2.COLOR_RGB2HSV)

        # Rango para naranja (ajustable segun tu iluminacion)
        lower_orange = (25, 100, 120)   #HSV min
        upper_orange = (160, 255, 255)    #HSV max

        mask = cv2.inRange(hsv, lower_orange, upper_orange)
        ratio = cv2.countNonZero(mask) / (mask.shape[0] * mask.shape[1] + 1e-5)

        if ratio > 0.1:
            print("Chaleco detectado")

        return ratio > 0.1  # Consideramos chaleco si mas del 25% del torso es naranja

    except Exception as e:
        print("Error detectando chaleco:", e)
        return False


# Iniciar el hilo de captura en modo daemon
threading.Thread(target=capture_thread, daemon=True).start()

# Lista de índices de puntos clave del torso (hombro izquierdo y cadera izquierda)
torso_keypoints = [11, 12, 15, 16, 23, 24]

# Variables de control
steeri = 0
speed = 0
hombro_iy = 0
cadera_iy = 360
distancia = 0
distancia_min = 2.3
distancia_max = 3.5
servo(130)
acel = 6
flagmanos = -1
manos_arriba_anterior = False
chaleconaranja = False


def main():
    global steer, steeri, speed, acel, hombro_iy, cadera_iy, distancia, chaleconaranja, distancia_min, distancia_max, velocidad_deseada, max_steer, new_frame, frame, flagmanos, manos_arriba_anterior

    # Configuración de MediaPipe Pose (configuración ultra-ligera)
    with mp_pose.Pose(
        model_complexity=0, #Probar con 0 y 1.       0 es optimizado, 1 confiable.
        min_detection_confidence=0.8,
        min_tracking_confidence=0.8
    ) as pose:
        prev_time = time.time()  # Para el cálculo de FPS
        
        while True:
            # Obtener el frame actual de forma segura
            with lock:
                if frame is None or not new_frame:
                    time.sleep(0.003)
                    continue
                bgr_frame = frame.copy()
                new_frame = False  # Resetea la bandera después de obtener el frame

            height, width, _ = bgr_frame.shape
            frame_center = width // 2
            
            rgb_frame = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2RGB)
            results = pose.process(rgb_frame)

            # Procesar la imagen con MediaPipe
            # results = pose.process(bgr_frame)
            
            cadera_iy = 360
            hombro_iy = 0

            if results.pose_landmarks:
				
                nuevo_chaleco = detectar_chaleco_naranja(rgb_frame, results.pose_landmarks.landmark, h=height, w=width)
                chaleconaranja = nuevo_chaleco
            
                # Obtener landmarks relevantes
                hombro_izq = results.pose_landmarks.landmark[11]
                muñeca_izq = results.pose_landmarks.landmark[15]

                hombro_der = results.pose_landmarks.landmark[12]
                muñeca_der = results.pose_landmarks.landmark[16]

                # Verificar si ambas manos están arriba con un margen del 10%
                margen = 0.1  # 10% de altura de imagen, # margen del 10% del alto de la imagen normalizada (0 a 1) - o + depende de orientacion, probar
                manos_arriba = (muñeca_izq.y < (hombro_izq.y - margen)) and (muñeca_der.y < (hombro_der.y - margen))
                #add chaleconaranja == True
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

                # Dibujar puntos clave del torso
                for i in torso_keypoints:
                    landmark = results.pose_landmarks.landmark[i]
                    x, y = int(landmark.x * width), int(landmark.y * height)
                    cv2.circle(bgr_frame, (x, y), 5, (0, 255, 0), -1)
            else:
                distancia = 1.0
                steer = 0
                chaleconaranja = False
                flagmanos = -1
                if speed > 0: 
                    speed -= acel*3
                
            if speed < 0:
                speed = 0
            # Mostrar valores en consola
            print(f"Steer = {steer:.1f}, Speed = {speed:.1f}, Distancia = {distancia:.1f} m, Flagmanos= {flagmanos}")
            send_command(steer, speed)

            # Calcular y mostrar FPS
            curr_time = time.time()
            fps = 1 / (curr_time - prev_time) if curr_time != prev_time else 0
            prev_time = curr_time
            cv2.putText(bgr_frame, f'FPS: {int(fps)}', (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)

            # Mostrar la imagen resultante
            cv2.imshow("Torso Tracking", bgr_frame)

            # Salir presionando 'q'
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    # Liberar recursos al salir
    picam2.stop()
    cv2.destroyAllWindows()
    
    pi.write(pin, 1) 
    pulso_gpio(PIN, DURACION)
    # Detén el servomotor y apaga la conexión de pigpio
    pi.set_servo_pulsewidth(gpio_pin, 0)
    pi.stop()
    if ser.is_open:
        ser.close()

if __name__ == '__main__':
    main()
