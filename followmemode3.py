#Codigo para seguir una persona, debe haber buena iluminacion, y los contornos de su ropa se deben diferenciar del ambiente.

import cv2
import mediapipe as mp
from picamera2 import Picamera2
import time
import threading
import numpy as np

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

def capture_thread():
    """Captura de imágenes en un hilo separado."""
    global frame, new_frame
    while True:
        temp_frame = picam2.capture_array()
        with lock:
            frame = temp_frame
            new_frame = True  # Se marca que hay un nuevo frame disponible

# Iniciar el hilo de captura en modo daemon
threading.Thread(target=capture_thread, daemon=True).start()

# Lista de índices de puntos clave del torso (hombro izquierdo y cadera izquierda)
torso_keypoints = [11, 12, 15, 16, 23, 24]

# Variables de control
steer = 0
speed = 0
hombro_iy = 0
cadera_iy = 360
distancia = 0
distancia_min = 1.5
velocidad_deseada = 100
acel = 4
flagmanos = -1
manos_arriba_anterior = False


# Filtro de Kalman para cadera izquierda (x, y)
kalman_cadera = cv2.KalmanFilter(4, 2)
kalman_cadera.measurementMatrix = np.array([[1, 0, 0, 0],
                                             [0, 1, 0, 0]], np.float32)
kalman_cadera.transitionMatrix = np.array([[1, 0, 1, 0],
                                           [0, 1, 0, 1],
                                           [0, 0, 1, 0],
                                           [0, 0, 0, 1]], np.float32)
kalman_cadera.processNoiseCov = np.eye(4, dtype=np.float32) * 0.03
kalman_cadera.measurementNoiseCov = np.eye(2, dtype=np.float32) * 0.5
kalman_initialized = False  # ⚠️ Bandera para inicialización


def main():
    global steer, speed, acel, hombro_iy, cadera_iy, distancia, new_frame, frame, flagmanos, manos_arriba_anterior, kalman_initialized

    # Configuración de MediaPipe Pose (configuración ultra-ligera)
    with mp_pose.Pose(
        model_complexity=0, #Probar con 0 y 1. 0 es optimizado, 1 confiable.
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
            
            #rgb_frame = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2RGB)
            #results = pose.process(rgb_frame)

            # Procesar la imagen con MediaPipe
            results = pose.process(bgr_frame)
            
            cadera_iy = 360
            hombro_iy = 0

            if results.pose_landmarks:
            
                # Obtener landmarks relevantes
                hombro_izq = results.pose_landmarks.landmark[11]
                muñeca_izq = results.pose_landmarks.landmark[15]

                hombro_der = results.pose_landmarks.landmark[12]
                muñeca_der = results.pose_landmarks.landmark[16]

                # Verificar si ambas manos están arriba con un margen del 5%
                margen = -0.1  # margen del 5% del alto de la imagen normalizada (0 a 1) - o + depende de orientacion, probar

                manos_arriba = (muñeca_izq.y > (hombro_izq.y - margen)) and (muñeca_der.y > (hombro_der.y - margen)) #> o < depende de orientacion, probar
                
                if manos_arriba != manos_arriba_anterior and manos_arriba == True:
                    flagmanos = -flagmanos
                    print(" Ambas manos están arriba")
                manos_arriba_anterior = manos_arriba
                
                # Extraer las coordenadas de la cadera izquierda (índice 23) y el hombro izquierdo (índice 11)
                cadera = results.pose_landmarks.landmark[23]
                cx = cadera.x * width
                cy = cadera.y * height
                medicion = np.array([[np.float32(cx)], [np.float32(cy)]], dtype=np.float32)

                if not kalman_initialized:
                    kalman_cadera.statePost = np.array([[cx], [cy], [0], [0]], dtype=np.float32)
                    kalman_initialized = True

                kalman_cadera.correct(medicion)
                prediccion = kalman_cadera.predict()

                cadera_ix_filtrada = prediccion[0][0]
                cadera_iy_filtrada = prediccion[1][0]
                cadera_iy = cadera_iy_filtrada
                cadera_ix = cadera_ix_filtrada
                


                hombro = results.pose_landmarks.landmark[11]
                hombro_iy = hombro.y * height

                # Evitar división por cero en caso poco probable de que ambos puntos estén al mismo nivel
                delta_y = abs(cadera_iy - hombro_iy)
                distancia = 200 / delta_y if delta_y != 0 else 0

                # Calcular steer en función de la posición horizontal de la cadera izquierda
                if flagmanos == 1: 
                    steer = max(min((cadera_ix - (width / 2)) * 0.2, 100), -100)

                else:
                    steer = 0
                    if speed > 0: 
                        speed -= acel*2

                # Actualizar la velocidad en función de la distancia y velocidad deseada
                if speed < velocidad_deseada and distancia > distancia_min and flagmanos == 1:
                    speed += acel
                if distancia <= distancia_min:
                    if speed > 0: 
                        speed -= acel

                # Dibujar puntos clave del torso
                for i in torso_keypoints:
                    landmark = results.pose_landmarks.landmark[i]
                    x, y = int(landmark.x * width), int(landmark.y * height)
                    cv2.circle(bgr_frame, (x, y), 5, (0, 255, 0), -1)
            else:
                distancia = 1.0
                steer = 0
                if speed > 0: 
                    speed -= acel
                
            if speed < 0:
                speed = 0
            # Mostrar valores en consola
            print(f"Steer = {steer:.1f}, Speed = {speed:.1f}, Distancia = {distancia:.1f} m, Flagmanos= {flagmanos}")

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

if __name__ == '__main__':
    main()
