import os
import cv2
from picamera2 import Picamera2
import time

# Inicializa la cámara
camera = Picamera2()
camera.configure(camera.create_preview_configuration(main={"size": (640, 480)}))
camera.start()

# Variable para la estación objetivo
target_station = None
    
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
            speak(f"Estación {target_station} seleccionada.")
            speak(f"Llevando la carga a la estación {target_station}")
            print(f"Estación {target_station} seleccionada.")
            break
        except ValueError:
            speak("Entrada no válida. Por favor, ingrese un número.")
            print("Entrada no válida. Por favor, ingrese un número.")

# Función para detectar el ArUco y verificar la estación
def detect_aruco():
    global target_station
    aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_50)
    aruco_params = cv2.aruco.DetectorParameters_create()

    
    print("Buscando marcadores ArUco...")
    while True:
        frame = camera.capture_array()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)

        if ids is not None:
            print(f"Marcadores detectados: {ids.flatten()}")
            if target_station in ids.flatten():
                speak(f"Estación {target_station} alcanzada. Deteniendo el robot.")
                print(f"Estación {target_station} alcanzada. Deteniendo el robot.")
                break  # Detén el robot aquí.

        # Agrega un pequeño retraso para evitar exceso de procesamiento
        time.sleep(0.1)

# Ciclo principal
try:
    select_station()
    if target_station is not None:
        detect_aruco()
    else:
        speak("No se seleccionó ninguna estación. Saliendo del programa.")
        print("No se seleccionó ninguna estación. Saliendo del programa.")
except KeyboardInterrupt:
    speak("El proceso fue interrumpido.")
    print("Proceso interrumpido.")
finally:
    camera.stop()
