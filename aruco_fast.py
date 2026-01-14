from picamera2 import Picamera2
import cv2
import numpy as np
from time import sleep
import time

# Configuración de la cámara
def setup_camera(resolution=(640, 480)):  # 640, 480     1280, 720     1920, 1080
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(main={"size": resolution})
    picam2.configure(config)
    #picam2.set_controls({"AwbEnable":False,"AeEnable":False,"AnalogueGain":4.0,"ExposureTime":190000,"ColourGains":(2.0,2.0)})
    picam2.set_controls({"ExposureTime": 6000, "AwbEnable": False, "ColourGains": (1.7, 1.4)})  #8000, 1.7, 1.4  Funciona bien de tarde en espacios cerrados.
    picam2.start()
    sleep(1)  # Esperar a que la cámara se estabilice
    return picam2
    
# Detección de ArUco
def detect_arucos(frame):
    # Convertir la imagen a escala de grises
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_50)  # Cambiar el diccionario según sea necesario
    parameters = cv2.aruco.DetectorParameters_create()
    #Ajustar los parámetros para mejorar la detección en movimiento
    parameters.adaptiveThreshWinSizeMin = 3
    parameters.adaptiveThreshWinSizeMax = 23
    parameters.adaptiveThreshWinSizeStep = 5
    #parameters.minMarkerPerimeterRate = 0.04  # Reducción de tamaño mínimo para capturar arucos en movimiento rápido
    #parameters.polygonalApproxAccuracyRate = 0.05
    parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    
    if ids is not None:
        frame = cv2.aruco.drawDetectedMarkers(frame, corners, ids)
    return frame, ids

def main():
    picam2 = setup_camera()
    marker_id = 0
    running = True

    try:
        while running:
            frame = picam2.capture_array()

            # Convertir de RGB a BGR para corregir los tonos azulados
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            
            # Girar 90 grados en sentido horario
            # frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
  
            # Detección de ArUco
            frame, ids = detect_arucos(frame)
            
            if ids is not None:
                for i in range(len(ids)):
                    # Procesar cada marcador ArUco detectado
                    marker_id = ids[i][0]
                    print(f"Detected ArUco Marker ID: {marker_id}")
                  
            cv2.imshow('Frame', frame)
            
            key = cv2.waitKey(10) & 0xFF
            if key == ord('q'):
                running = False

    finally:
        picam2.close()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
