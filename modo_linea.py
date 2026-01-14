from picamera2 import Picamera2
import cv2
import numpy as np
from time import sleep
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
MAX_STEER = 80  # Límite máximo para la dirección (steer).
MAX_SPEED = 80  # Límite máximo para la velocidad (speed).

# Variable para la estación objetivo
target_station = None

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
    
# Función para seleccionar la estación con teclado
def select_station():
    global target_station
    speak("Seleccione la estación a donde debo llevar la carga.")
    print("Seleccione la estación a donde debe llevar la carga:")
    print("Presione un número del 1 al 9 y luego Enter.")
    while True:
        try:
            user_input = input("Estación: ")
            target_station = int(user_input)
            #speak(f"Estación {target_station} seleccionada.")
            speak(f"Llevando la carga a la estación {target_station}")
            print(f"Estación {target_station} seleccionada.")
            break
        except ValueError:
            speak("Entrada no válida. Por favor, ingrese un número.")
            print("Entrada no válida. Por favor, ingrese un número.")

# Inicialización de la conexión serial
try:
    # Pulso inicial al iniciar el programa
    print("Encendiendo GPIO al inicio...")
    pulso_gpio(PIN, duracion)
    
    ser = serial.Serial('/dev/ttyAMA0', 115200, timeout=2)  # Configuración del puerto serial de la Raspberry Pi a 115200 baudios con un timeout de 1 segundo.
    # Inicializa el puerto y limpia el buffer
    ser.reset_input_buffer()
    
except serial.SerialException as e:
    # Si ocurre un error al abrir el puerto, se muestra el mensaje de error y se detiene el programa.
    print(f"Error al abrir el puerto serial: {e}")
    exit(1)

    # Configuración de la cámara
def setup_camera(resolution=(640, 360)):  # 640, 480     1280, 720     1920, 1080
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(main={"size": resolution})
    picam2.configure(config)
    #picam2.set_controls({"AwbEnable":False,"AeEnable":False,"AnalogueGain":4.0,"ExposureTime":190000,"ColourGains":(2.0,2.0)})
    picam2.set_controls({"AfMode": 2})  #8000, 1.7, 1.4  Funciona bien de tarde en espacios cerrados.
    picam2.start()
    sleep(1)  # Esperar a que la cámara se estabilice
    return picam2

# Procesamiento de imagen
def preprocess_image(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    _, binary = cv2.threshold(blurred,50, 255, cv2.THRESH_BINARY_INV)

    # Crear un kernel para operaciones morfológicas
    kernel = np.ones((5, 5), np.uint8)

    # Aplicar erosión para hacer más burda la línea
    eroded = cv2.erode(binary, kernel, iterations=1)

    # Aplicar dilatación para adelgazar los elementos estructurales
    dilated = cv2.dilate(eroded, kernel, iterations=1)

    return dilated

def detect_line(binary_frame):
    frame_height, frame_width = binary_frame.shape
    roi = binary_frame[int(frame_height / 2):, :]
    contours, _ = cv2.findContours(roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    return contours, roi, frame_width, frame_height

def compute_steer(cx, frame_center):
    return ((cx - frame_center) / frame_center) * MAX_STEER
    
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
        
# Detección de ArUco
def detect_arucos(frame):
    # Convertir la imagen a escala de grises
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_50)  # Cambiar el diccionario según sea necesario
    parameters = cv2.aruco.DetectorParameters()
    #Ajustar los parámetros para mejorar la detección en movimiento
    parameters.adaptiveThreshWinSizeMin = 3
    parameters.adaptiveThreshWinSizeMax = 23
    parameters.adaptiveThreshWinSizeStep = 5
    parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
    corners, ids, _ = detector.detectMarkers(gray)
    
    if ids is not None:
        frame = cv2.aruco.drawDetectedMarkers(frame, corners, ids)
    return frame, ids

# Variables globales para PID
steerPID = 0.0
prev_steer = 0.0  # Variable para almacenar el valor anterior de 'steer'

# Función para implementar el control PID
def pid_control(steer, kp=0.25, kd=0.01, dt=0.1):
    global steerPID, prev_steer
    
    # Componente proporcional
    steerP = steer

    # Componente derivativa (derivada aproximada)
    steerD = (steer - prev_steer) / dt

    # PID final
    steerPID = (steerP * kp) + (steerD * kd)

    # Limitar Steer entre -500 y 500
    steerPID = int(max(-MAX_STEER, min(MAX_STEER, steerPID)))

    # Actualizar el valor previo de 'steer'
    prev_steer = steer

    return steerPID
    

def main():
    
    picam2 = setup_camera()
    
    steer_output = 0
    speed_output = 0
    marker_id = 0  # Ubicacion aruco ID
    last_send_time = 0  # Ultima vez que se envio comandos de control de motores

    acel = 2   #Aceleracion, 2 rpm/ciclo.
    areamin = 3000 #Area minima de la linea   3000
    areamax = 30000 #Area maxima de la linea    30000
    timesteerhold = 1 #Steer 0 despues de 1 segundo de perder la linea
    
    running = True
    last_send_timeLine = 0  #Ultima vez que se vio la linea

    # Inicializar la última posición del centroide como el centro del frame
    last_cx = None
    
    select_station()

    try:
        while running:
            
            frame = picam2.capture_array()

            # Convertir de RGB a BGR para corregir los tonos azulados
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            
            # Girar 90 grados en sentido horario
            # frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)

            binary_frame = preprocess_image(frame)
            contours, roi, frame_width, frame_height = detect_line(binary_frame)
            roi_offset = int(frame_height / 2)
            frame_center = frame_width // 2

            # Detección de líneas
            if contours:
                largest_contour = max(contours, key=cv2.contourArea)
                M = cv2.moments(largest_contour)
                if areamax > M['m00'] > areamin:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00']) + roi_offset
                    steer = compute_steer(cx, frame_center)

                    # Dibuja la línea y el círculo en el centroide detectado
                    cv2.line(frame, (cx, cy), (frame_center, cy), (0, 255, 0), 6)
                    cv2.circle(frame, (cx, cy), 8, (0, 0, 255), -1)

                    # Guardar la última posición del centroide
                    last_cx = cx

                    steer_output = pid_control(steer)
                    last_send_timeLine = time.time()
                    speed_output += acel

                else:
                    if speed_output > 0:
                        speed_output -= acel

                    print("Línea no detectada")
                    # Evaluar si la línea se perdió hacia la izquierda o derecha
                    if last_cx is not None:
                        if last_cx < frame_center:
                            print("Perdida a la izquierda")     
                            steer_output = steerPID
                        else:
                            print("Perdida a la derecha")
                            steer_output = steerPID
            else:
                if speed_output > 0:
                    speed_output -= acel

                print("Línea no detectada")
                # Evaluar si la línea se perdió hacia la izquierda o derecha
                if last_cx is not None:
                    if last_cx < frame_center:
                        print("Perdida a la izquierda")
                        steer_output = steerPID
                    else:
                        print("Perdida a la derecha")
                        steer_output = steerPID

            current_time = time.time()
            # Si ha pasado un tiempo de 3s desde que se perdió la línea, se establece steer_output en 0.
            if current_time - last_send_timeLine >= timesteerhold:
                steer_output = 0
             
            speed_output = int(max(-MAX_SPEED, min(MAX_SPEED, speed_output)))
            
            
            
            if marker_id == target_station:
                speak(f"Estación {target_station} alcanzada. Deteniendo el robot.")
                print(f"Estación {target_station} alcanzada. Deteniendo el robot.")
                steer_output = 0
                speed_output = 0
                select_station()
                
                
                
            print(f"Steer: {steer_output}      Speed: {speed_output}      Ubication: {marker_id}")
            
            # Enviar comandos continuamente
            current_time = time.time()
            # Si ha pasado el tiempo definido en TIME_SEND (50 ms), se envía un nuevo comando.
            if current_time - last_send_time >= TIME_SEND:
                send_command(steer_output, speed_output)  # Enviar los valores actuales de dirección y velocidad.
                last_send_time = current_time  # Actualizar el tiempo del último envío.

            # Detección de ArUco
            frame, ids = detect_arucos(frame)
            
            if ids is not None:
                for i in range(len(ids)):
                    # Procesar cada marcador ArUco detectado
                    marker_id = ids[i][0]
                    #print(f"Detected ArUco Marker ID: {marker_id}")
                  
                        
            cv2.drawContours(frame[roi_offset:, :], contours, -1, (0, 255, 0), 3) # Contornos de linea
            cv2.imshow('Frame', frame)
            
            # Un retardo de 10 ms es suficiente para evitar ciclos innecesarios y mantener el programa eficiente.
            # time.sleep(0.01)
            
            key = cv2.waitKey(10) & 0xFF
            if key == ord('q'):
                running = False

    finally:
        picam2.close()
        
        speak("El proceso fue interrumpido.")
        print("Proceso interrumpido.")
        
        # Pulso final antes de salir
        pulso_gpio(PIN, duracion)
        # Limpieza de los GPIO
        GPIO.cleanup()
        
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
