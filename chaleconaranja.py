from picamera2 import Picamera2
import cv2
import numpy as np
import time

def nothing(x):
    pass

# Inicializar Picamera2
picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"format": 'RGB888', "size": (640, 480)})
picam2.configure(config)
picam2.start()

# Esperar a que la cmara est lista
time.sleep(2)

# Crear ventana con sliders para ajustar HSV
cv2.namedWindow("Control HSV", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Control HSV", 400, 300)

cv2.createTrackbar("H min", "Control HSV", 0, 179, nothing)
cv2.createTrackbar("H max", "Control HSV", 25, 179, nothing)
cv2.createTrackbar("S min", "Control HSV", 100, 255, nothing)
cv2.createTrackbar("S max", "Control HSV", 255, 255, nothing)
cv2.createTrackbar("V min", "Control HSV", 100, 255, nothing)
cv2.createTrackbar("V max", "Control HSV", 255, 255, nothing)

while True:
    frame = picam2.capture_array()
    hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)

    # Leer valores de los sliders
    h_min = cv2.getTrackbarPos("H min", "Control HSV")
    h_max = cv2.getTrackbarPos("H max", "Control HSV")
    s_min = cv2.getTrackbarPos("S min", "Control HSV")
    s_max = cv2.getTrackbarPos("S max", "Control HSV")
    v_min = cv2.getTrackbarPos("V min", "Control HSV")
    v_max = cv2.getTrackbarPos("V max", "Control HSV")

    lower_orange = np.array([h_min, s_min, v_min])
    upper_orange = np.array([h_max, s_max, v_max])

    # Crear la mscara y aplicar
    mask = cv2.inRange(hsv, lower_orange, upper_orange)
    result = cv2.bitwise_and(frame, frame, mask=mask)

    # Mostrar resultados
    cv2.imshow("Original", frame)
    cv2.imshow("Mascara Naranja", mask)
    cv2.imshow("Resultado", result)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        print("\n?? Rango HSV final para chaleco naranja:")
        print(f"lower_orange = np.array([{h_min}, {s_min}, {v_min}])")
        print(f"upper_orange = np.array([{h_max}, {s_max}, {v_max}])")
        break

picam2.stop()
cv2.destroyAllWindows()
